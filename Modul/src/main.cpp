
#define MOTOR yes
#define SPI_protocol yes

#include <Arduino.h>
#include <Wire.h>

#include "config.h" // Основной конфигурационный файл с общими настройками

// Файлы с функциями отдельных сущностей
#include "motor.h"
#include "code.h"
#include "protokolSPI.h"

void setup()
{
    pinMode(2, OUTPUT); // Временно для отладки
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, 1);

    Serial.begin(115200);
    delay(500);
    Serial.println(String(millis()) + " Start program ESP32_Body ...");

    initInterrupt(); // Инициализация прерывания на концевиках
    initTimer_0();   // Запуск таймера 0

#ifdef MOTOR
    initMotor();          // Начальная инициализация и настройка шаговых моторов
    setSpeedMotor(SPEED); // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение
    initTimer_1();        // Запускаем после того как установили скоростьи посчитали интервал импульсов. Таймер на моторы, один на все так как управляем по положению я макисмально возможной скоростью передвижения

    // setZeroMotor();       // Установка в ноль
    // digitalWrite(PIN_Motor_En, 0); // 0- Разрешена работа 1- запрещена работа драйвера
    // printf("countPulse= %i \n", countPulse);
    // int32_t aa = micros();
    // digitalWrite(PIN_Motor_En, 0); // Включаем драйвера

    // testMotorRun();
    // for (int i = 0; i < 4; i++)
    // {
    //     setMotorAngle(0, 90);
    //     setMotorAngle(1, 90);
    //     setMotorAngle(2, 90);
    //     setMotorAngle(3, 90);
    // }
    // delay(1000);

    // printf("motor[0].position= %i \n", motor[0].position);

    // int32_t bb = micros();
    // printf("time run micros= %i \n", bb - aa);
    // printf("countPulse= %i \n", countPulse);

    // testMotorStop();

    // initTimer_2(); //Таймер на 2 мотор
    // setSpeed_R(0.5);
    // setSpeed_L(0.5);

    // while (1)
    //     ;

    // stopMotor();
#endif

#ifdef SPI_protocol
    initSPI_slave(); // Инициализация SPI_slave
    Serial.println("spi_slave_queue_Send... ");
    spi_slave_queue_Send(); // Configure receiver Первый раз закладываем данные чтобы как только мастер к нам обратиться было чем ответить
#endif

    Serial.println(String(millis()) + " End SetUp !!!");
    digitalWrite(PIN_LED, 0);
}

void loop()
{
    // digitalWrite(PIN_ANALIZ, 1);
    //----------------------------- 50 миллисекунд --------------------------------------
    if (flag_timer_50millisec)
    {
        flag_timer_50millisec = false;
        // flag_data = true; // Есть новые данные по шине // РУчной вариант имитации пришедших данных с частотой 20Гц
    }


    //----------------------------- По факту обмена данными с верхним уровнем --------------------------------------
    if (flag_data) // Если обменялись данными
    {
        flag_data = false;
        printf("+\n");

        processingDataReceive(); // Обработка пришедших данных после состоявшегося обмена  !!! Подумать почему меняю данные даже если они с ошибкой, потом по факту когда будет все работать
                                 // Data2Modul_receive.command = 0; // В ручном режиме имитирую приход нужной команды
        executeDataReceive();    // Выполнение пришедших команд

        // printf(" Receive id= %i cheksum= %i All obmen= %i bed_time= %i bed_crc= %i", Data2Driver_receive.id, Data2Driver_receive.cheksum, obmen_all, obmen_bed_time, obmen_bed_crc);
        // printf(" \n");

        collect_Data_for_Send(); // Собираем данные в структуре для отправки
        spi_slave_queue_Send();  // Закладываем данные в буфер для передачи(обмена)
    }

    //----------------------------- 10 миллисекунд --------------------------------------
    if (flag_timer_10millisec)
    {
        flag_timer_10millisec = false;
    }

    //----------------------------- 1 секунда --------------------------------------
    if (flag_timer_1sec) // Вызывается каждую секунду
    {
        flag_timer_1sec = false;
        disableMotor(); // Отключение моторов при простое

        printf(" %f  \n", millis() / 1000.0); // Печать времени что программа не зависла, закомментировать в реальной работе
        for (int i = 0; i < 4; i++)
        {
            // printf(" motor %i position %i destination %i status %i \n", i, motor[i].position, motor[i].destination, motor[i].status);
        }

        // printBody();
    }

    Led_Blink(PIN_LED, 500); // Мигаем светодиодом каждую 1/2 секунду, что-бы было видно что цикл не завис
    // digitalWrite(PIN_ANALIZ, 0);
}

//***********************************************************************
