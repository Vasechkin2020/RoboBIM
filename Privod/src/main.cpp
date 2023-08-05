
#define SPI_protocol_def yes
#define microDS3231_def yes

#include <Arduino.h>
#include <Wire.h>

#include "microDS3231.h" // Чужая библиотека https://github.com/GyverLibs/microDS3231
MicroDS3231 rtc(0x68);   // адрес по умолчанию 0x68

#include "config.h"

#include "i2c_my.h"
#include "protokolSPI.h"
#include "ds3231.h" // чип времени

#include "code.h"

void setup()
{
    delay(2000);
    initLed(); // Начальная инициализация и настройка светодиодов
    pinMode(PIN_ANALIZ, OUTPUT);
    pinMode(15, INPUT_PULLDOWN);

    Serial.begin(115200);
    Serial2.begin(115200);
    Serial.println(" ");
    Serial.println(String(micros()) + " Start program ESP32_Body ...");

    initTimer_0(); // Запуск таймера 0

    // initialize EEPROM with predefined size
    // EEPROM.begin(EEPROM_SIZE);

    Wire.begin(); // Старт шины I2C
    Wire.setClock(400000);
    Serial.println(String(micros()) + " Start scan  I2C ...");
    scanI2C();
    delay(3000);
    // Serial.println(String(micros()) + " Start scan  After TCA9548A ...");
    // for (uint8_t i = 0; i < 8; i++)
    // {
    //     set_TCA9548A(i);
    //     Serial.print("---");
    //     Serial.print(" Slot = ");
    //     Serial.println(i);
    //     scanI2C();
    // }
    // delay(1000);

#ifdef microDS3231_def
    init_DS3231(); // Инициализация чипа времени
#endif

#ifdef SPI_protocol_def
    printf("initSPI_slave \n");
    initSPI_slave(); // Инициализация SPI_slave
    printf("spi_slave_queue_Send \n");
    spi_slave_queue_Send(); // Configure receiver Первый раз закладываем данные чтобы как только мастер к нам обратиться было чем ответить
#endif

    offLed(); // Отключение светодиодов
    Serial.println(String(micros()) + " End SetUp.");
    delay(1000);
}

int a, b;

void loop()
{
    // printf("+\n");
    digitalWrite(PIN_ANALIZ, 1);
    Led_Blink(PIN_LED_BLUE, 500); // Мигаем светодиодом каждую секунду, что-бы было видно что цикл не завис
    //Led_Blink(PIN_LED_RED, 500);  // Мигаем светодиодом каждую секунду, что-бы было видно что цикл не завис

    //----------------------------- 10 миллисекунд --------------------------------------
    if (flag_timer_10millisec)
    {
        flag_timer_10millisec = false;
        // Serial.print("10 ");
        // flag_data = true; // Зависает esp32 если включить при подключенной малинке
    }

    //----------------------------- По факту обмена данными с верхним уровнем --------------------------------------
    if (flag_data) // Если обменялись данными
    {
        flag_data = false;
        //  printf("+\n");
        processing_Data(); // Обработка пришедших данных после состоявшегося обмена
        //  executeCommand();  // Выполнение пришедших команд

        printf(" Receive id= %i cheksum= %i All obmen= %i bed_time= %i bed_crc= %i \n ", Data2Iot_receive.id, Data2Iot_receive.cheksum, obmen_all, obmen_bed_time, obmen_bed_crc);
        // printf(" command= %i radius= %f speed= %f motor_video_angle= %f \n", Data2Iot_receive.command_body, Data2Iot_receive.radius, Data2Iot_receive.speed, Data2Iot_receive.motor_video_angle);
        // printf(" \n");

#ifdef SPI_protocol_def
        collect_Data();         // Собираем данные в структуре для отправки
        spi_slave_queue_Send(); // Закладываем данные в буфер для передачи(обмена)
#endif
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
        printf(" %f \n", millis() / 1000.0);

#ifdef microDS3231_def
        getTime_DS3231();
        printTime_DS3231();
        // getTemperature_DS3231();
#endif

        // printf(".");

        // Serial.print(".");
        // printBody();

        // Serial.println(b-a);

        //  Serial.println(b - a);
    }
    //----------------------------- 1 МИНУТА!!!! --------------------------------------
    if (flag_timer_60sec) // Вызывается каждую МИНУТУ
    {
        flag_timer_60sec = false;
#ifdef INA219_def
        // Ina219.calculateCapacity(); // колибровка в ноль и сохранение в память
#endif

        // Печать времени что программа не зависла, закомментировать в реальной работе
        printf(" %f \n", millis() / 1000.0);
    }
    digitalWrite(PIN_ANALIZ, 0);
}

//***********************************************************************
