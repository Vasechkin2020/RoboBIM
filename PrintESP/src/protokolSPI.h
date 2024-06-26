#ifndef PROTOCOLSPI_H
#define PROTOCOLSPI_H

#include "driver/spi_master.h"
#include "driver/spi_slave.h"

esp_err_t ret;
spi_transaction_t t;

//************** MASTER ********************************
#define TX_HOST HSPI_HOST

// Определение пинов SPI для ESP32 HSPI_HOST
#define PIN_NUM_MISO_HSPI 12
#define PIN_NUM_MOSI_HSPI 13
#define PIN_NUM_CLK_HSPI 14
#define PIN_NUM_CS_HSPI 15

static const char *TAG = "SPI_Master";

spi_device_handle_t handle;

void initSPI_master()
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI_HSPI,
        .miso_io_num = PIN_NUM_MISO_HSPI,
        .sclk_io_num = PIN_NUM_CLK_HSPI,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    spi_device_interface_config_t devcfg = {
        .mode = 0, // SPI mode 0
        //.duty_cycle_pos = 128, //50% duty cycle
        .clock_speed_hz = 10 * 1000 * 1000, // Clock out at 4 MHz
        .spics_io_num = PIN_NUM_CS_HSPI,    // CS pin
        .queue_size = 7,                    // We want to be able to queue 7 transactions at a time
    };

    // Инициализация SPI шины
    ret = spi_bus_initialize(TX_HOST, &buscfg, 2);
    ESP_ERROR_CHECK(ret);

    // Присоединение SPI устройства к шине
    ret = spi_bus_add_device(TX_HOST, &devcfg, &handle);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SPI setup completed.");
}

//************** SLAVE ********************************
#define RX_HOST VSPI_HOST
#define DMA_CHAN_RX 2

#define PIN_NUM_CS 5
#define PIN_NUM_CLK 18
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23

spi_slave_transaction_t rx_start_transaction;
spi_slave_transaction_t *rx_end_transaction;

int obmen_all = 0;
int obmen_bed_time = 0;
int obmen_bed_crc = 0; // Подсчет успешных и нет обменов по SPI

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
uint32_t diff = 0;
IRAM_ATTR void rx_ok(spi_slave_transaction_t *trans) // IRAM_ATTR Добавить так чтобы в другой памяти были функции ???
{

    // Иногда из-за помех, звона или чего-то еще мы получаем два IRQ друг за другом. Это решается с помощью
    // смотреть на время между прерываниями и отказываться от любого прерывания слишком близко к другому.
    //  https://github.com/espressif/esp-idf/blob/f8bda324ecde58214aaa00ab5e0da5eea9942aaf/examples/peripherals/spi_slave/sender/main/app_main.c

    static uint32_t last_isr_time = 0;
    uint32_t currtime = micros();
    diff = currtime - last_isr_time;
    last_isr_time = currtime; // Запоминаем время правильного прерывания

    if (diff < 1000) // Если новое прерывание возникло рашьше чем 1000 микросекунд (1 милисекунды)
    {
        // printf(" diff= %i",diff);
        flag_goog_data_time = false;
        obmen_bed_time++;
    }
    else
    {
        obmen_all++; // Считаем сколько было обменов данными
        flag_goog_data_time = true;
    }
    flag_data = true; // Флаг что обменялись данными
}

void initSPI_slave()
{
    Serial.println("Control Slave_SPI init !");
    pinMode(PIN_NUM_MISO, OUTPUT); // Линия на выход
    // ПО умолчанию все к минусу кроме чипселект
    pinMode(PIN_NUM_MOSI, INPUT); // Линия на вход подтянута к минусу
    pinMode(PIN_NUM_CLK, INPUT);  // Линия на вход подтянута к минусу
    // pinMode(PIN_NUM_MOSI, INPUT_PULLDOWN); // Линия на вход подтянута к минусу
    // pinMode(PIN_NUM_CLK, INPUT_PULLDOWN);  // Линия на вход подтянута к минусу

    pinMode(PIN_NUM_CS, INPUT_PULLUP); // Линия на вход подтянута к Плюсу

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
    rx_start_transaction.tx_buffer = &Print2Data_send;  // Ссылка на структуру которую отправляем
    ret = spi_slave_queue_trans(RX_HOST, &rx_start_transaction, portMAX_DELAY);
    assert(ret == ESP_OK);
}

// Обработка пришедших данных после срабатывания прерывания что обмен состоялся
void processing_Data()
{
    ret = spi_slave_get_trans_result(RX_HOST, &rx_end_transaction, portMAX_DELAY); // Wait for received data
    assert(ret == ESP_OK);

    if (flag_goog_data_time) // Если прерывание было вовремя, а не случайное
    {
        Struct_Data2Print *copy_buf_slave_receive = (Struct_Data2Print *)buf_slave_receive; // Создаем переменную в которую пишем адрес буфера в нужном формате
        Data2Print_receive = *copy_buf_slave_receive;                           // Копируем из этой перемнной данные в мою структуру
        uint32_t cheksum_receive = measureCheksum(Data2Print_receive);          // Считаем контрольную сумму пришедшей структуры

        if (cheksum_receive != Data2Print_receive.cheksum || Data2Print_receive.cheksum == 0)
        {
            obmen_bed_crc++;
        }
    }
}

void sendSPI(uint8_t *arrSend_, uint8_t size_)
{
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = size_ * 8;     // Length is in bits (48 bytes * 8 bits/byte)
    t.tx_buffer = arrSend_;   // Data
    // ret = spi_device_transmit(handle, &t); // Transmit! // Начало передачи данных по SPI
    ret = spi_device_queue_trans(handle, &t, portMAX_DELAY); // Transmit! // Начало передачи данных по SPI
    assert(ret == ESP_OK);                                   // Check if transmission was successful
}

#endif