
#define MOTOR yes
#define SPI_protocol yes
#define LASER yes

#include <Arduino.h>
#include <Wire.h>

#include "config.h" // Основной конфигурационный файл с общими настройками

#include "sk60plus.h"
CSk60plus sk60plus[4]; // 4 лазерных датчика
// #include "laser80M.h"
// CLaser80M laser80m; // Переменная для лазера

// Файлы с функциями отдельных сущностей
#include "motor.h"
#include "code.h"
#include "protokolSPI.h"

bool flag = false;

void setup()
{
    // pinMode(2, OUTPUT); // Временно для отладки

    //pinMode(PIN_LED, OUTPUT);
    //digitalWrite(PIN_LED, 1);

    Serial.begin(921600);
    Serial2.begin(115200);

    delay(500);
    Serial.println(String(millis()) + "Start ESP32_Modul printBIM(c) 2024 printBIM.com 2106");

    initTimer_0();   // Запуск таймера 0
    
    initInterrupt(); // Инициализация прерывания на концевиках и сопоставление по моторам
    setSpeedMotor(SPEED); // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение
    initTimer_1();        // Запускаем после того как установили скоростьи посчитали интервал импульсов. Таймер на моторы, один на все так как управляем по положению я макисмально возможной скоростью передвижения

    sk60plus[0].autoBaund(); //  Установка скорости работы датчиков
    delay(999);
    sk60plus[0].init(0x00);
    sk60plus[1].init(0x01);
    sk60plus[2].init(0x02);
    sk60plus[3].init(0x03);
    delay(999);

    // sk60plus[3].setLaser(0x01);
    // delay(999);

    // sk60plus[0].setModulAddress(0X01);

#ifdef MOTOR
    initMotor();          // Начальная инициализация и настройка шаговых моторов
    // testMotorRun();
    // testMicric();

    setZeroMotor(); // Установка в ноль
    // digitalWrite(PIN_Motor_En, 0); // 0- Разрешена работа 1- запрещена работа драйвера

#endif

#ifdef SPI_protocol
    initSPI_slave(); // Инициализация SPI_slave
    Serial.println("spi_slave_queue_Send... ");
    spi_slave_queue_Send(); // Configure receiver Первый раз закладываем данные чтобы как только мастер к нам обратиться было чем ответить
#endif

    Serial.println(String(millis()) + " End SetUp !!!");
    // digitalWrite(PIN_LED, 0);
    flag = true;
    delay(1000);
    timeSpi = millis(); // Запоминаем время обмена
    //sk60plus[2].setModulMeasureOffset(0x02);
}

void loop()
{
// digitalWrite(PIN_ANALIZ, 1);
#ifdef LASER

  if (Data2Modul_receive.controlLaser.mode == 1) // Если идет команда 1 то работаем
  {
    laserLoop(); // Обработка датчиков так что-бы не задерживать основной цикл и делать все короткими операциями
    //printf("laserLoop... \n");
  }
#endif

//----------------------------- По факту обмена данными с верхним уровнем --------------------------------------
#ifdef SPI_protocol
    if (flag_data) // Если обменялись данными
    {
        flag_data = false;
        timeSpi = millis(); // Запоминаем время обмена
        // printf("+\n");
        collect_Data_for_Send(); // Собираем данные в структуре для отправки на момент прихода команлы, но БЕЗ учета команды.До исполнения команды.
        processingDataReceive(); // Обработка пришедших данных после состоявшегося обмена  !!! Подумать почему меняю данные даже если они с ошибкой, потом по факту когда будет все работать
        // printf(" mode= %i \n",Data2Modul_receive.controlMotor.mode);
        executeDataReceive();    // Выполнение пришедших команд

        // printf(" Receive id= %i cheksum= %i command= %i ", Data2Modul_receive.id, Data2Modul_receive.cheksum,Data2Modul_receive.command );
        // printf(" All= %i bed= %i ", spi.all, spi.bed);
        // printf(" angle0= %.2f angle1= %.2f angle2= %.2f angle3= %.2f", Data2Modul_receive.angle[0], Data2Modul_receive.angle[1], Data2Modul_receive.angle[2], Data2Modul_receive.angle[3] );

        spi_slave_queue_Send();  // Закладываем данные в буфер для передачи(обмена)
    }
#endif

if (millis() - timeSpi > 5000) //Если обмена нет больше 5 секунд то отключаем все
{
    Data2Modul_receive.controlLaser.mode = 0;  //Отключаем лазерные датчики
    Data2Modul_receive.controlMotor.mode = 0;  //Отключаем моторы
    digitalWrite(PIN_Motor_En, 1); // Выключаем драйвера
}

    //----------------------------- 10 миллисекунд --------------------------------------
    if (flag_timer_10millisec)
    {
        flag_timer_10millisec = false;
    }
    //----------------------------- 50 миллисекунд --------------------------------------
    if (flag_timer_50millisec)
    {
        flag_timer_50millisec = false;
        // flag_data = true; // Есть новые данные по шине // РУчной вариант имитации пришедших данных с частотой 20Гц
    }

    //----------------------------- 1 секунда --------------------------------------
    if (flag_timer_1sec) // Вызывается каждую секунду
    {
        flag_timer_1sec = false;

        //byte addr = 0x80;

        if (flag)
        {
            flag = false;
            // laser80m.setTimeInterval(0);
            // laser80m.setStartingPoint(1);
            // laser80m.setFrequency(5);
            // laser80m.setResolution(1);
            // laser80m.setRange(30);
            // laser80m.setDistanceModification(-19);
            // delay(100);
        }

        // sk60plus[0].getBroadcastSingleMeasure();
        // delay(250);
        // for (int i = 0; i < 25; i++)
        // {
        //     sk60plus.readStatus();
        //     //delay(10);
        // }
        // while (!sk60plus[0].readStatus())
        // {
        //     delay(1);
        // }
        // sk60plus[0].readMeasureResult();
        // sk60plus.stopContinuous();
        // delay(50);
        // sk60plus.startContinuousSuperFast();
        // laser80m.singleMeasurement(addr);

        // controlLaser(0,80);
        // byte buf[5] = {0x80, 0x06, 0x05, 0x01, 0x74};
        // sizeof(buf);
        // inversBit(buf);
        // byte cS = buf[0] + buf[1] + buf[2] + buf[3];
        // Serial.print(cS,DEC);
        // Serial.print(" ");
        // Serial.print(cS,HEX);
        // Serial.print(" ");
        // Serial.println(cS,BIN);
        // byte ret = (cS ^ 0b11111111) + 1;
        // Serial.print(ret,DEC);
        // Serial.print(" ");
        // Serial.print(ret,HEX);
        // Serial.print(" ");
        // Serial.println(ret,BIN);
        // byte ret = (byte)data_ ^ 0b11111111;
        // Serial.println(ret, BIN);

        // stopMeasurement(addr);
        //  broadcastMeasurement();
        // delay(777); // Время когда будет готов результат измерения
        // readCache(addr);

        
        //disableMotor(); // Отключение моторов при простое//Возможно это не правильно и моторы отключаются раньше чем следовало бы из за энерции перелетают дальше....

        // sk60plus[0].startSingleFast();
        // sk60plus[1].startSingleFast();
        // sk60plus[2].startSingleFast();

        //sk60plus[3].startSingleFast();

        printf(" %.3f  \n", millis() / 1000.0); // Печать времени что программа не зависла, закомментировать в реальной работе
        for (int i = 0; i < 4; i++)
        {
            // printf(" motor %i position %i destination %i status %i \n", i, motor[i].position, motor[i].destination, motor[i].status);
            //printf(" laser %i _distance %i _signalQuality %i \n", i, sk60plus[i]._distance, sk60plus[i]._signalQuality);
        }

        // printBody();
    }

    // Led_Blink(PIN_LED, 500); // Мигаем светодиодом каждую 1/2 секунду, что-бы было видно что цикл не завис
    //  digitalWrite(PIN_LED, 0);
}

//***********************************************************************
