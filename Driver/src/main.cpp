
#include <Arduino.h>
#include <Wire.h>

#define MOTOR yes
#define BNO_def yes
#define SPI_protocol yes
//  #define LED_def yes
// #define VL530L0X_def yes
// #define RCWL1601_def yes

#include "config.h" // Основной конфигурационный файл с общими настройками
#include "code.h"

int dd = 0;

void setup()
{
    // initialize EEPROM with predefined size
    EEPROM.begin(EEPROM_SIZE);
    // Запуск таймера 0
    initTimer_0();
    // Сразу отключение драйвера моторов чтобы не гудел
    pinMode(PIN_En, OUTPUT);
    digitalWrite(PIN_En, 1); // 0- Разрешена работа 1- запрещена работа драйвера
    // Настройка скорости порта UART1
    Serial.begin(115200);
    Serial.println(" ");
    Serial.println(String(millis()) + " Start ESP32_Driver printBIM(c) 2024 printBIM.com");
    // Начальная инициализация и настройка светодиодов
    initLed();

    Wire.begin(); // Старт шины I2C
    Wire.setClock(400000);
    Serial.println(String(millis()) + " Start init I2C 400000 ...");

    
    //scanI2C();//Поиск устройств на шине I2C
    // set_TCA9548A(0);
    // Serial.print(" All = ");
    // scanI2C();
    // Serial.println(" === ");
    //  for (uint8_t i = 0; i < 8; i++)
    //  {
    //      set_TCA9548A(i);
    //      Serial.print(" Slot = ");
    //      Serial.println(i);
    //      delay(100);
    //      scanI2C();
    //      delay(100);
    //  }

    // delay(1000000);

#ifdef MOTOR
    initMotor(); // Начальная инициализация и настройка шаговых моторов
    // testMotor(); // Функция тестирования правильности подключения моторов.
#endif

#ifdef VL530L0X_def
    Init_VL53L0X(Sensor_VL53L0X_L, 0x29, multi_line_VL53L0X_L); // Инициализация датчиков сверху
    Init_VL53L0X(Sensor_VL53L0X_R, 0x29, multi_line_VL53L0X_R); // Инициализация датчиков сверху
#endif

#ifdef LED_def
    led2812.init(); // Инициализация светодиодов
    led2812.test();
    delay(1000);
#endif

#ifdef RCWL1601_def
    // Инициализация портов для ультразвукового датчика
    init_Uzi();
#endif

#ifdef BNO_def
    Setup_BNO055();
    delay(999);
#endif

    // #ifdef SPI_protocol
    //  Иницируем всегда, иначе ошибки на шине
    initSPI_slave();        // Инициализация SPI_slave
    spi_slave_queue_Send(); // Configure receiver Первый раз закладываем данные чтобы как только мастер к нам обратиться было чем ответить

    // printf("spi_slave_queue_Send \n");
    // Тут надо подготовить структуру с 0 айди для первогораза отправки
    // #endif

    printf("%lu End SetUp \n", millis());
}

int a, b;
float ppp = 0;

void loop()
{
    // digitalWrite(PIN_ANALIZ, 1);
    Led_Blink(PIN_LED_GREEN, 500); // Мигаем светодиодом каждую 1/2 секунду, что-бы было видно что цикл не завис

#ifdef SPI_protocol
    //----------------------------- По факту обмена данными с верхним уровнем --------------------------------------
    if (flag_data) // Если обменялись данными
    {
        flag_data = false;
        // !!! Сделать постоянную калибровку гироскопа если стоим на месте. скоростьравна нулю и считаем какое-то время. если вдруг поехали то ничего не меняем, а если успели откалиброваться то меняем офсеты.
        processing_Data();      // Обработка пришедших данных после состоявшегося обмена
        executeCommand();       // Выполнение пришедших команд
        BNO055_readData();     // Опрашиваем датчик получаем углы
        calcEncod();            // По энкодерам снимаем показания
        collect_Driver2Data();  // Собираем данные в структуре для отправки
        spi_slave_queue_Send(); // Закладываем данные в буфер для передачи(обмена)
    }
#endif
    //----------------------------- 10 миллисекунд --------------------------------------
    if (flag_timer_10millisec)
    {
        flag_timer_10millisec = false;
#ifdef MOTOR // Контроль за моторами по условиям
             // getKoefAccel();      // Расчет коэффициента ускорений при движении по радиусу
             // setAcceleration_L(); // Контроль за ускорением мотора
             // setAcceleration_R(); // Контроль за ускорением мотора
#endif

#ifdef BNO_def

#endif

#ifdef RCWL1601_def
        loopUzi(); // Все действия по ультразвуковому датчику

        // inTimerUzi(); // Проверка на случай если сигнал не вернется тогда через заданное время сбросим (было так раньше, потом перенес в основную функцию loopUzi())
#endif
    }
    //----------------------------- 50 миллисекунд --------------------------------------
    if (flag_timer_50millisec)
    {
        flag_timer_50millisec = false;
        // Время исполнения всех вычислений по датчику 5 милисекунд
         //calculateOdom_imu(); // Подсчет одометрии по imu

#ifdef MOTOR
        movementTime(); // Отслеживание времени движения
        delayMotor();   // Задержка отключения драйверов моторов после остановки

#endif

#ifdef RCWL1601_def
        if (!Flag_uzi_wait) // Если прошлый сигнал обработали и не ждем данные
        {
            Triger_start(); // Запускаем новый цикл измерения ультразвуковым датчиком каждые 50 милисекунд
        }
#endif
#ifdef VL530L0X_def
        loop_VL53L0X(); // Измерение лазерными датчиками
#endif
    }
    //----------------------------- 1 СЕКУНДА !!!!  --------------------------------------
    if (flag_timer_1sec) // Вызывается каждую секунду
    {
        flag_timer_1sec = false;
        // BNO055_getCalibration();
        // dd++;
        // if (dd > 10)
        // {
        //     // Тест как будто мы уточняем координаты и скорости. То что СУММИРУЕТ ТО НУЖНО УТОЧНЯТЬ НЕ МЕНЕЕ 1 раза за секунду
        //     bno055.pose.x = 0; // Вычисляем координаты
        //     bno055.pose.y = 0; // Вычисляем координаты
        //     bno055.vel.vx = 0;
        //     bno055.vel.vy = 0;
        //     dd = 0;
        // }

        // printf("%li spi: all %i bed %i \n", millis(), spi.all, spi.bed);
        // printf("\n");

#ifdef RCWL1601_def
        // Serial.print(" distance = ");
        // Serial.print(distance_uzi);
#endif

#ifdef VL530L0X_def
        // Serial.print(" distance laserL= ");
        // Serial.print(laser_L);
        // Serial.print(" distance laserR= ");
        // Serial.println(laser_R);
#endif
    }

    // digitalWrite(PIN_ANALIZ, 0);
}
