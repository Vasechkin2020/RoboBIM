
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
    delay(2000);
    initLed(); // Начальная инициализация и настройка светодиодов
    pinMode(PIN_ANALIZ, OUTPUT);

    Serial.begin(115200);
    Serial2.begin(115200);
    Serial.println(" ");
    Serial.println(String(micros()) + " Start program ESP32_Body ...");

    initTimer_0(); // Запуск таймера 0

    // hspi = new SPIClass(HSPI);
    // hspi->begin();
    // pinMode(hspi->pinSS(), OUTPUT); // HSPI SS

    // Wire.begin(); // Старт шины I2C
    // Wire.setClock(400000);
    // Serial.println(String(micros()) + " Start scan  I2C ...");
    // scanI2C();
    delay(1000);

    initSPI_master(); // Инициализация SPI master для отправки данных

#ifdef SPI_protocol_def
    printf("initSPI_slave \n");
    initSPI_slave(); // Инициализация SPI_slave
    printf("spi_slave_queue_Send \n");
    spi_slave_queue_Send(); // Configure receiver Первый раз закладываем данные чтобы как только мастер к нам обратиться было чем ответить
#endif

    // kartr.printLine(line10, 10);
    // Serial.println(String(micros()) + "AAAA " + line10[0]);
    // printf("ppppp");
    // kartr.line10to150(line10, line150);
    // kartr.printLine(line150, 150);
    kartr.line15toCode24(line15_1, code24);
    kartr.code24toCode48(code24, code48);

    kartr.line15toCode24(line15_0, code24null);
    kartr.code24toCode48(code24null, code48null);

    kartr.code24toCode48(code24free, code48free);

    Serial.println(" === ");
    for (int i = 0; i < 24; i++)
    {
        Serial.print(" i= ");
        Serial.print(i);
        Serial.print(" = ");
        Serial.println(code24free[i], HEX);
    }
    // u_int8_t *code48 = (uint8_t *)code24; // Создаем переменную в которую пишем адрес буфера в нужном формате
    //                                       // code48Send = *code48;                // Копируем из этой перемнной данные в мою структуру

    Serial.println(" =code48= ");
    for (int i = 0; i < 48; i++)
    {
        Serial.print(" i= ");
        Serial.print(i);
        Serial.print(" = ");
        Serial.println(code48free[i], HEX);
    }

    offLed(); // Отключение светодиодов
    Serial.println(String(micros()) + " End SetUp.");
    delay(1000);
}

int a, b;

void loop()
{

    // printf("+\n");
    digitalWrite(PIN_ANALIZ, 1);
    // Led_Blink(PIN_LED_BLUE, 500); // Мигаем светодиодом каждую секунду, что-бы было видно что цикл не завис
    Led_Blink(PIN_LED_RED, 500); // Мигаем светодиодом каждую секунду, что-бы было видно что цикл не завис

    //----------------------------- 10 миллисекунд --------------------------------------
    if (flag_timer_10millisec)
    {
        flag_timer_10millisec = false;

        // digitalWrite(hspi->pinSS(), LOW); // pull SS slow to prep other end for transfer
        // hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));

        // for (int i = 0; i < 48; i++)
        //     hspi->transfer(code48[i]);

        // hspi->endTransaction();
        // digitalWrite(hspi->pinSS(), HIGH); // pull ss high to signify end of data transfer

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

        printf(" Receive id= %i cheksum= %i All obmen= %i bed_time= %i bed_crc= %i \n ", Data2Print_receive.id, Data2Print_receive.cheksum, obmen_all, obmen_bed_time, obmen_bed_crc);
        // printf(" command= %i radius= %f speed= %f motor_video_angle= %f \n", Data2Print_receive.command_body, Data2Print_receive.radius, Data2Print_receive.speed, Data2Print_receive.motor_video_angle);
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

        printLine(line15_1,36);
        printLine(line15_0,36);
        printLine(line15_4,500);
        printLine(line15_1,36);
        printLine(line15_0,36);
        printLine(line15_2,36);
        printLine(line15_3,36);
        printLine(line15_2,36);
        printLine(line15_0,36);
        printLine(line15_1,36);

        sendSPI(code48free, 48); // Какой массив отправлять и размер массива                // Check if transmission was successful

        printf(" %f \n", millis() / 1000.0);
    }
    //----------------------------- 1 МИНУТА!!!! --------------------------------------
    if (flag_timer_60sec) // Вызывается каждую МИНУТУ
    {
        flag_timer_60sec = false;
        // Печать времени что программа не зависла, закомментировать в реальной работе
        printf(" %f \n", millis() / 1000.0);
    }
    digitalWrite(PIN_ANALIZ, 0);
}

//***********************************************************************
