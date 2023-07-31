
// #define MOTOR yes
// #define REMOTEXY yes
// #define BNO_def yes
// #define SPI_protocol yes
// #define INA219_def yes // Датчик по питанию сервомоторов
// #define LED_def yes
#define MOTOR_SERVO_def yes
// #define VL530L0X_def yes
// #define RCWL1601_def yes

#include <Arduino.h>
#include <Wire.h>

#include "config.h" // Основной конфигурационный файл с общими настройками

// Файлы с функциями отдельных сущностей
#include "i2c_my.h"
#include "ina219.h"
#include "my_wifi.h"
#include "my_remotexy.h"
#include "motor.h"
#include "protokolSPI.h"
#include "bno.h"
#include "lx16a.h"
#include "led43.h"
led43 led2812; // Экземпляр класса

#include "VL53L0X.h" // Чужая библиотека polulu https://github.com/pololu/vl53l0x-arduino
VL53L0X Sensor_VL53L0X_L;
VL53L0X Sensor_VL53L0X_R;

#include "code.h"
#include "uzi.h"


void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println(" ");
    Serial.println(String(micros()) + " Start program ESP32_Driver RoboBIM ...");

#ifdef MOTOR
    initMotor(); // Начальная инициализация и настройка шаговых моторов
    // setSpeed_L(0.3);
    // setSpeed_R(0.3);
    // delay(5000);
    // stopMotor();
#endif

    initLed(); // Начальная инициализация и настройка светодиодов

    // initialize EEPROM with predefined size
    EEPROM.begin(EEPROM_SIZE);

    Serial.println(String(millis()) + " Start init I2C ...");
    Wire.begin(); // Старт шины I2C
    Wire.setClock(400000);
    Serial.println(String(millis()) + " End init I2C ...");

    scanI2C();
    delay(1000);
#ifdef VL530L0X_def
    // Sensor_VL53L0X_R.newAddress(0x30);
    // Sensor_VL53L0X_R.setAddress(0x31);
    // Wire.begin(); // Старт шины I2C

    printf("Init_VL53L0X_Left \n");
    Init_VL53L0X(Sensor_VL53L0X_L, 0x30); // Инициализация датчиков сверху
    printf("Init_VL53L0X_Right \n");
    Init_VL53L0X(Sensor_VL53L0X_R, 0x31); // Инициализация датчиков сверху
#endif

#ifdef REMOTEXY
    Serial.println("init_WiFi...");
    init_WiFi(); // Инициализация Вайфай в нужных режимах

    Serial.println("Start Remote XY...");
    RemoteXY_Init(); // Инициализируем библиотеку, внутри отключил создание сети и прочее не нужное в файле lib\RemoteXY\src\modules\espcore_wifi.h в 41 строке
#endif
    //  delay(20000);

#ifdef MOTOR_SERVO_def
    initServo(); // Начальная инициализация и настройка сервомоторов
    testServo(); // Начальная проверка сервомоторов
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
#endif

    printf("initTimer_0 \n");
    initTimer_0(); // Запуск таймера 0

#ifdef SPI_protocol
    printf("initSPI_slave \n");
    initSPI_slave(); // Инициализация SPI_slave
    printf("spi_slave_queue_Send \n");
    spi_slave_queue_Send(); // Configure receiver Первый раз закладываем данные чтобы как только мастер к нам обратиться было чем ответить
#endif

#ifdef INA219_def
    init_INA219(); // Инициализация датчика тока
    getIna219();   // Считывание значений при запуске
    printDataIna219();
    delay(3000);
#endif

    // delay(50000000);

    // offLed(); // Отключение светодиодов

    delay(1000);
    // flag_data = true; // Зависает esp32 если включить при подключенной малинке
    Serial.println(String(millis()) + " End SetUp !!!");
}

int a, b;

void loop()
{
    // digitalWrite(PIN_ANALIZ, 1);
    if (flag_setup == 0)
    {
        Led_Blink(PIN_LED_GREEN, 500); // Мигаем светодиодом каждую 1/2 секунду, что-бы было видно что цикл не завис
    }
    else
    {
        Led_Blink(PIN_LED_GREEN, 100); // Мигаем светодиодом каждую 1/2 секунду, что-бы было видно что цикл не завис
    }

#ifdef MOTOR
    movementTime(); // Отслеживание времени движения
    delayMotor();   // Задержка отключения драйверов моторов после остановки
#endif
#ifdef REMOTEXY
    RemoteXY_Handler(); // Обработчик. считывает данные и хранит во внутренних переменных.
#endif

    //----------------------------- По факту обмена данными с верхним уровнем --------------------------------------
    if (flag_data) // Если обменялись данными
    {
        flag_newData = 1; // Есть новые данные по шине
        flag_data = false;
        // printf("+\n");
        processing_Data(); // Обработка пришедших данных после состоявшегося обмена

        // printf(" Receive id= %i cheksum= %i All obmen= %i bed_time= %i bed_crc= %i", Data2Driver_receive.id, Data2Driver_receive.cheksum, obmen_all, obmen_bed_time, obmen_bed_crc);
        // printf(" command= %i radius= %f speed= %f motor_video_angle= %f \n", Data2Driver_receive.command_body, Data2Driver_receive.radius, Data2Driver_receive.speed, Data2Driver_receive.motor_video_angle);
        // printf(" \n");

        // calculateOdom_enc();    // Подсчет одометрии по энкодерам
        // calculateOdom_imu();    // Подсчет одометрии по imu
        collect_Data();         // Собираем данные в структуре для отправки
        spi_slave_queue_Send(); // Закладываем данные в буфер для передачи(обмена)
    }

    //----------------------------- 10 миллисекунд --------------------------------------
    if (flag_timer_10millisec)
    {
        flag_timer_10millisec = false;

#ifdef REMOTEXY
        if (RemoteXY.connect_flag == 1) // Если есть связь то берем данные от ручного управления и заменяем те что пришли по шине
        {
            changeDataFromRemoteXY(); // Заменяем данные, если мы на ручном управлении
        }
#endif
        // Serial.println(String(millis()) + " flag_newData = " + flag_newData);
        // Serial.println(String(millis()) + " flag_newControlData = "+ flag_newControlData);
        if (flag_newData || flag_newControlData) // Выполняем если есть новые данные или по шине или вручную если не будет команд ни по шине не вручную то робот остановиться
        {
            executeCommand();        // Выполнение пришедших команд
            flag_newData = 0;        // Сброс флага
            flag_newControlData = 0; // Сброс флага
        }

#ifdef BNO_def
        BNO055_readEuler();  // Опрашиваем датчик получаем углы
        BNO055_readLinear(); // Опрашиваем датчик получаем ускорения
#endif

#ifdef INA219_def
        getIna219(); // Считывание показаний датчика INA219
                     //  printDataIna219(); // Печать показаний датчика INA219
#endif
#ifdef RCWL1601_def
        loopUzi(); // Все действия по ультразвуковому датчику
        //inTimerUzi(); // Проверка на случай если сигнал не вернется тогда через заданное время сбросим (было так раньше, потом перенес в основную функцию loopUzi())
#endif
    }

    //----------------------------- 50 миллисекунд --------------------------------------
    if (flag_timer_50millisec)
    {
        flag_timer_50millisec = false;
        // printRemoteXY();
        // setSpeed_time(0.2, 0.2, 1000);
#ifdef RCWL1601_def
        if (!Flag_uzi_wait) // Если прошлый сигнал обработали и не ждем данные
        {
            Triger_start(); // Запускаем новый цикл измерения ультразвуковым датчиком каждые 50 милисекунд
        }
#endif
    }

    //----------------------------- 1 секунда --------------------------------------
    if (flag_timer_1sec) // Вызывается каждую секунду
    {
        flag_timer_1sec = false;

        printf(" %f \n", millis() / 1000.0);
        // Ina219.printData(); // Вывод на печать данных
        printDataIna219();

        Serial.print(" distance = ");
        Serial.println(distance_uzi);

#ifdef VL530L0X_def
        loop_VL53L0X(); // Измерение лазерными датчиками
#endif
        // long a = micros();
        // long b = micros();

        // Serial.print(b - a);
        // Serial.println(" time");

        // printBody();

        // Serial.println("setSpeed_time");
    }

    if (flag_timer_60sec) // Вызывается каждую МИНУТУ
    {
        flag_timer_60sec = false;
#ifdef INA219_def
        // Ina219.calculateCapacity(); // колибровка в ноль и сохранение в память
#endif

        // Печать времени что программа не зависла, закомментировать в реальной работе
        printf(" %f \n", millis() / 1000.0);
    }
    // digitalWrite(PIN_ANALIZ, 0);
}

//***********************************************************************
