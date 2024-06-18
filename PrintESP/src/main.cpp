
#define SPI_protocol_def yes

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "config.h"

#include "g1309s.h" // Класс для кодирования картриджа
CG1309s kartr;      // Обьект класса картридж

#include "i2c_my.h"
#include "protokolSPI.h"
#include "code.h"

void setup()
{
    Serial.begin(115200);
    Serial.println(" ");
    Serial.println(String(millis()) + " Start program ESP32_Body ...");

    initPIN(); // Начальная инициализация и настройка светодиодов

    initTimer_0(); // Запуск таймера 0
    //initTimer_1(); // Таймер на 1 мотор

    // hspi = new SPIClass(HSPI);
    // hspi->begin();
    // pinMode(hspi->pinSS(), OUTPUT); // HSPI SS

    delay(100);

#ifdef SPI_protocol_def
    printf("initSPI_slave \n");
    initSPI_slave(); // Инициализация SPI_slave
    printf("spi_slave_queue_Send \n");
    spi_slave_queue_Send(); // Configure receiver Первый раз закладываем данные чтобы как только мастер к нам обратиться было чем ответить
#endif

    //initSPI_master(); // Инициализация SPI master для отправки данных
    initLineArray();  // Формирование массива с разными строками печати

    offPIN(); // Отключение светодиодов
    Serial.println(String(millis()) + " End SetUp.");
    // delay(1000);
}

void loop()
{
    // digitalWrite(PIN_ANALIZ, 1);
    //----------------------------- По факту обмена данными с верхним уровнем --------------------------------------
    if (flag_data) // Если обменялись данными
    {
        flag_data = false;
        if (flag_goog_data_time) // Если прерывание было вовремя, а не случайное
        {
            timer_connection = millis(); // Есть правильная связь запоминаем время
            processing_Data();           // Обработка пришедших данных после состоявшегося обмена
            executeCommand();            // Выполнение пришедших команд

            // printf(" Receive id= %i cheksum= %i All obmen= %i bed_time= %i bed_crc= %i \n ", Data2Print_receive.id, Data2Print_receive.cheksum, obmen_all, obmen_bed_time, obmen_bed_crc);
            // printf("+ \n");
        }

#ifdef SPI_protocol_def
        collect_Data();         // Собираем данные в структуре для отправки
        spi_slave_queue_Send(); // Закладываем данные в буфер для передачи(обмена)
#endif
    }

    if ((millis() - timer_connection) > 1000) // Если связь пропала больше 1 секунды назад то отключаем печать
        //controlPrint.status = 0;

    //----------------------------- 10 миллисекунд --------------------------------------
    if (flag_timer_10millisec)
    {
        flag_timer_10millisec = false;
    }
    //----------------------------- 50 миллисекунд --------------------------------------
    if (flag_timer_50millisec)
    {
        flag_timer_50millisec = false;
    }
    //----------------------------- 1 секунда --------------------------------------
    if (flag_timer_1sec) // Вызывается каждую секунду
    {
        flag_timer_1sec = false;
        // printf(" %f \n", millis() / 1000.0);
    }
    // if (digitalRead(36) == 0)
    // {
    //     controlPrint.status = 1;
    //     controlPrint.mode = 4;
    // }
    // else
    // {
    //     controlPrint.status = 0;
    //     controlPrint.mode = 0;
    // }
    Led_Blink(PIN_LED_BLUE, 500); // Мигаем светодиодом каждую секунду, что-бы было видно что цикл не завис
    // Led_Blink(PIN_LED_RED, 500); // Мигаем светодиодом каждую секунду, что-бы было видно что цикл не завис
    // digitalWrite(PIN_ANALIZ, 0);
}
