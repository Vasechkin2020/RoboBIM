#ifndef PROTOCOLSPI_H
#define PROTOCOLSPI_H

#include <driver/spi_master.h>
#include "driver/spi_slave.h"

#define RX_HOST VSPI_HOST
#define DMA_CHAN_RX 2

#define PIN_NUM_CS 5
#define PIN_NUM_CLK 18
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23

esp_err_t ret;

spi_slave_transaction_t rx_start_transaction;
spi_slave_transaction_t *rx_end_transaction;

volatile bool flag_data = false;           // Флаг что данные передались
volatile bool flag_goog_data_time = false; // Флаг что данные пришли через правильное время

#define SIZE_BUFF 192 // Размер буфера который передаем. Следить что-бы структуры не превышали этот размер Кратно 32 делать
// WORD_ALIGNED_ATTR unsigned char buf_slave_receive[SIZE_BUFF]; // Буфер в 1 kByte
WORD_ALIGNED_ATTR unsigned char buf_slave_receive[SIZE_BUFF]; // Буфер в  Byte

//*************************************************************************************

IRAM_ATTR void ready_tx(spi_slave_transaction_t *trans)
{
    // Serial.println(String(micros()) + " New data ready no SEND !");
    // digitalWrite(OUT_LATCH_PIN, HIGH);
    // digitalWrite(OUT_LATCH_PIN, LOW);
}

IRAM_ATTR void rx_ok(spi_slave_transaction_t *trans) // IRAM_ATTR Добавить так чтобы в другой памяти были функции ???
{
    spi.all++; // Считаем сколько было обменов данными всего

    // Иногда из-за помех, звона или чего-то еще мы получаем два IRQ друг за другом. Это решается с помощью
    // смотреть на время между прерываниями и отказываться от любого прерывания слишком близко к другому.
    //  https://github.com/espressif/esp-idf/blob/f8bda324ecde58214aaa00ab5e0da5eea9942aaf/examples/peripherals/spi_slave/sender/main/app_main.c

    static uint32_t last_isr_time;
    uint32_t currtime = micros();
    uint32_t diff = currtime - last_isr_time;
    last_isr_time = currtime; // Запоминаем время правильного прерывания

    if (diff < 1000) // Если новое прерывание возникло рашьше чем 1000 микросекунд (1 милисекунды)
    {
        flag_goog_data_time = false;
        spi.bed++;
    }
    else
    {
        flag_goog_data_time = true;
    }
    flag_data = true; // Флаг что обменялись данными
}

void initSPI_slave()
{
    Serial.println("Control Slave_SPI init...");
    pinMode(PIN_NUM_MISO, OUTPUT); // Линия на выход
    // ПО умолчанию все к минусу кроме чипселект
    pinMode(PIN_NUM_MOSI, INPUT_PULLDOWN); // Линия на вход подтянута к минусу
    pinMode(PIN_NUM_CLK, INPUT_PULLDOWN);  // Линия на вход подтянута к минусу
    pinMode(PIN_NUM_CS, INPUT_PULLUP);     // Линия на вход подтянута к Плюсу

    // При одиночном подключении в малинке, без другого модуля ошибки единичные
    // pinMode(PIN_NUM_MOSI, INPUT); // Линия на вход подтянута к минусу резистором на плате поэтому подтяжка не нужна
    // pinMode(PIN_NUM_CLK, INPUT);  // Линия на вход подтянута к минусу резистором на плате поэтому подтяжка не нужна
    // pinMode(PIN_NUM_CS, INPUT);   // Линия на вход подтянута к Плюсу резистором на плате поэтому подтяжка не нужна

    // Configuration for the RX SPI bus
    spi_bus_config_t rx_bus_config = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
    };

    spi_slave_interface_config_t rx_slave_config = {
        .spics_io_num = PIN_NUM_CS,
        .flags = 0,
        .queue_size = 3,
        .mode = SPI_MODE0,
        .post_setup_cb = ready_tx,
        .post_trans_cb = rx_ok,
    };
    // Initialize SPI slave interface
    ret = spi_slave_initialize(RX_HOST, &rx_bus_config, &rx_slave_config, 1);
    assert(ret == ESP_OK);
}

// Функция в которой чистим буфер и закладываем данные на передачу в буфер
void spi_slave_queue_Send()
{
    memset(&rx_start_transaction, 0, sizeof(rx_start_transaction));
    memset(buf_slave_receive, 0, sizeof(buf_slave_receive));
    rx_start_transaction.length = SIZE_BUFF * 8 + 32;   // Колличество бит в транзакции с запасом
    rx_start_transaction.rx_buffer = buf_slave_receive; // Буфер памяти куда получим структуру
    rx_start_transaction.tx_buffer = &Modul2Data_send;  // Ссылка на структуру которую отправляем
    ret = spi_slave_queue_trans(RX_HOST, &rx_start_transaction, portMAX_DELAY);
    assert(ret == ESP_OK);
}

// Обработка пришедших данных после срабатывания прерывания что обмен состоялся
void processingDataReceive()
{
    Struct_Data2Modul Data2Modul_receive_temp;                     // Экземпляр структуры получаемых данных временный, пока не посчитаем контроьную сумму и убедимся что данные хорошие
    ret = spi_slave_get_trans_result(RX_HOST, &rx_end_transaction, portMAX_DELAY); // Wait for received data
    assert(ret == ESP_OK);
    // printf("=\n");
    if (flag_goog_data_time) // Если прерывание было вовремя, а не случайное
    {
        Struct_Data2Modul *copy_buf_slave_receive = (Struct_Data2Modul *)buf_slave_receive; // Создаем переменную в которую пишем адрес буфера в нужном формате
        Data2Modul_receive_temp = *copy_buf_slave_receive;                                       // Копируем из этой перемнной данные в мою структуру
        uint32_t cheksum_receive = measureCheksum(Data2Modul_receive_temp);                      // Считаем контрольную сумму пришедшей структуры

        if (cheksum_receive != Data2Modul_receive_temp.cheksum || Data2Modul_receive_temp.cheksum == 0) // Стравниваю что сам посчитал и что прислали. Не сходится или ноль - значит плохие данные
        {
            spi.bed++; // Плохие данные
        }
        else
        {
            Data2Modul_receive = Data2Modul_receive_temp; // Хорошие данные копируем
        }
    }
}

#endif