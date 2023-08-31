
#include <Arduino.h>
#include <Wire.h>

// #define MOTOR yes
//  #define REMOTEXY yes
#define BNO_def yes
#define SPI_protocol yes
//  #define LED_def yes
#define MOTOR_SERVO_def yes
#define VL530L0X_def yes
#define RCWL1601_def yes

#include "config.h" // Основной конфигурационный файл с общими настройками
#include "code.h"

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
    Serial.println(String(micros()) + " Start program ESP32_Driver RoboBIM ...");
    // Начальная инициализация и настройка светодиодов
    initLed();

    Serial.println(String(millis()) + " Start init I2C ...");
    Wire.begin(); // Старт шины I2C
    Wire.setClock(400000);
    Serial.println(String(millis()) + " End init I2C ...");

    // Поиск устройств на шине I2C
    scanI2C();
    // set_TCA9548A(0);
    // scanI2C();
    // for (uint8_t i = 0; i < 5; i++)
    // {
    //     set_TCA9548A(i);
    //     Serial.print(" Slot = ");
    //     Serial.println(i);
    //     delay(1000);
    //     scanI2C();
    //     delay(1000);
    // }

    // delay(1000000);

#ifdef MOTOR
    initMotor(); // Начальная инициализация и настройка шаговых моторов
    // setSpeedRPS_L(0.5);
    // setSpeedRPS_R(0.5);
    // delay(3000);
    // stopMotor();

    // setSpeedRPS_L(1);
    // setSpeedRPS_R(1);
    // delay(50000000);
#endif

#ifdef VL530L0X_def
    Init_VL53L0X(Sensor_VL53L0X_L, 0x29, multi_line_VL53L0X_L); // Инициализация датчиков сверху
    Init_VL53L0X(Sensor_VL53L0X_R, 0x29, multi_line_VL53L0X_R); // Инициализация датчиков сверху
#endif

#ifdef REMOTEXY
    Serial.println("init_WiFi...");
    init_WiFi(); // Инициализация Вайфай в нужных режимах
    Serial.println("Start Remote XY...");
    RemoteXY_Init(); // Инициализируем библиотеку, внутри отключил создание сети и прочее не нужное в файле lib\RemoteXY\src\modules\espcore_wifi.h в 41 строке
    //  delay(20000);
#endif

#ifdef MOTOR_SERVO_def
    initServo(); // Начальная инициализация и настройка сервомоторов
    // getServoId(); //
    // changeServoId(2,1); // Смена id сервомотора
    setStartPosition(341, 311);
    // delay(3000);
    //  setStartPosition(0,0); // Для начальной установки стоек чтобы не задекали ничего
    //  delay(3000000);

    // delay(999);
    // testServo2();
    // testServo(); // Начальная проверка сервомоторов
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

#ifdef SPI_protocol
    initSPI_slave(); // Инициализация SPI_slave
    // printf("spi_slave_queue_Send \n");
    // Тут надо подготовить структуру с 0 айди для первогораза отправки
    spi_slave_queue_Send(); // Configure receiver Первый раз закладываем данные чтобы как только мастер к нам обратиться было чем ответить
#endif

    Serial.println(String(millis()) + " ++++++++++++++++++++++++++++++++++++++++ End SetUp !!! +++++++++++++++++++++++++++++++++++++++");
}

int a, b;
float ppp = 0;

void loop()
{
    // digitalWrite(PIN_ANALIZ, 1);
    blink_led();

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
        //  printf(" command= %i radius= %f speed= %f motor_video_angle= %f \n", Data2Driver_receive.command_body, Data2Driver_receive.radius, Data2Driver_receive.speed, Data2Driver_receive.motor_video_angle);
        //  printf(" \n");

        // printData2Driver_receive();

        collect_Driver2Data();  // Собираем данные в структуре для отправки
        spi_slave_queue_Send(); // Закладываем данные в буфер для передачи(обмена)
    }

    //***********************************************************************

    millisec_10();
    millisec_50();
    sec_1();
    min_1();

    // digitalWrite(PIN_ANALIZ, 0);
}
