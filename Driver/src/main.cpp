
#include <Arduino.h>
#include <Wire.h>

#define MOTOR yes
//#define BNO_def yes
#define SPI_protocol yes
// #define LED_def yes
// #define VL530L0X_def yes
// #define RCWL1601_def yes

#include "config.h" // Основной конфигурационный файл с общими настройками
#include "code.h"

// int dd = 0;

void setup()
{
    // initialize EEPROM with predefined size
    EEPROM.begin(EEPROM_SIZE);
    // Запуск таймера 0
    initTimer_0();
    // Сразу отключение драйвера моторов чтобы не гудел
    pinMode(PIN_En, OUTPUT);
    digitalWrite(PIN_En, 1); // 0- Разрешена работа 1- запрещена работа драйвера

    pinMode(Pla_PIN_En, OUTPUT);
    digitalWrite(Pla_PIN_En, 1); // 0- Разрешена работа 1- запрещена работа драйвера

    // Настройка скорости порта UART1
    Serial.begin(115200);
    Serial.println(" ");
    Serial.println(String(millis()) + " Start ESP32_Driver printBIM(c) 2024 printBIM.com");
    // Начальная инициализация и настройка светодиодов
    // initLed(); Не включать так как совпадает с моторами на платформу

    Wire.begin(); // Старт шины I2C
    Wire.setClock(400000);
    Serial.println(String(millis()) + " Start init I2C 400000 ...");

    // scanI2C();//Поиск устройств на шине I2C
    //  set_TCA9548A(0);
    //  Serial.print(" All = ");
    //  scanI2C();
    //  Serial.println(" === ");
    //   for (uint8_t i = 0; i < 8; i++)
    //   {
    //       set_TCA9548A(i);
    //       Serial.print(" Slot = ");
    //       Serial.println(i);
    //       delay(100);
    //       scanI2C();
    //       delay(100);
    //   }

    // delay(1000000);

#ifdef MOTOR
    initMotor(); // Начальная инициализация и настройка шаговых моторов

    // digitalWrite(Pla_PIN_En, 0); // 0- Разрешена работа 1- запрещена работа драйвера
    // digitalWrite(Pla_PIN_L_Dir, 0); //
    // digitalWrite(Pla_PIN_R_Dir, 0); //
    // flag_motor_PLA = 1;

    // testMotor(); // Функция тестирования правильности подключения моторов.
#endif

#ifdef LED_def
    led2812._init(); // Инициализация светодиодов
    led2812._test();
    // delay(100000);
#endif

#ifdef VL530L0X_def
    Init_VL53L0X(Sensor_VL53L0X_L, 0x29, multi_line_VL53L0X_L); // Инициализация датчиков сверху
    Init_VL53L0X(Sensor_VL53L0X_R, 0x29, multi_line_VL53L0X_R); // Инициализация датчиков сверху
#endif

#ifdef RCWL1601_def
    // Инициализация портов для ультразвукового датчика
    init_Uzi();
#endif

#ifdef BNO_def
    Setup_BNO055();
    delay(500);
#endif

    // #ifdef SPI_protocol
    //  Иницируем всегда, иначе ошибки на шине
    initSPI_slave();        // Инициализация SPI_slave
    collect_Driver2Data();  // Собираем данные в структуре для отправки
    spi_slave_queue_Send(); // Configure receiver Первый раз закладываем данные чтобы как только мастер к нам обратиться было чем ответить

    // printf("spi_slave_queue_Send \n");
    // Тут надо подготовить структуру с 0 айди для первогораза отправки
    // #endif

    printf("%lu End SetUp \n", millis());
}

// int a, b;
// float ppp = 0;

void loop()
{
    // digitalWrite(PIN_ANALIZ, 1);
    // Led_Blink(PIN_LED_GREEN, 500); // Мигаем светодиодом каждую 1/2 секунду, что-бы было видно что цикл не завис
    ledBlink(36, 42, 500); // Мигаем светодиодом каждую 1/2 секунду, что-бы было видно что цикл не завис

#ifdef SPI_protocol
    //----------------------------- По факту обмена данными с верхним уровнем --------------------------------------
    if (flag_data) // Если обменялись данными
    {
        flag_data = false;
        // !!! Сделать постоянную калибровку гироскопа если стоим на месте. скоростьравна нулю и считаем какое-то время. если вдруг поехали то ничего не меняем, а если успели откалиброваться то меняем офсеты.
        processing_Data(); // Обработка пришедших данных после состоявшегося обмена
        executeCommand();  // Выполнение пришедших команд
#ifdef BNO_def
        BNO055_readData(); // Опрашиваем датчик получаем углы
#endif
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

#ifdef RCWL1601_def
        loopUzi(); // Все действия по ультразвуковому датчику
                   // inTimerUzi(); // Проверка на случай если сигнал не вернется тогда через заданное время сбросим (было так раньше, потом перенес в основную функцию loopUzi())
#endif
#ifdef LED_def
        led2812._show(); // Передаем данные на светодиоды 100 Герц
#endif
    }
    //----------------------------- 50 миллисекунд --------------------------------------
    if (flag_timer_50millisec)
    {
        flag_timer_50millisec = false;
        // Время исполнения всех вычислений по датчику 5 милисекунд
        // calculateOdom_imu(); // Подсчет одометрии по imu

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
        // platforma(); //Выравнивание платформы
       
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
        // Serial.print(uzi.distance);
        // Serial.print(" status = ");
        // Serial.println(uzi.status);
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
/*

m_Grid.SetItemText(i,  3, ftos(paper.m_dLastС));
m_Grid.SetItemText(i,  4, GetStrDTime(paper.m_sysLastDTime));

m_Grid.SetItemText(i,  5, ftos(paper.m_dDivC2*m_MainThread.m_dFactorC+paper.m_dDivX2*m_MainThread.m_dFactorX, 2));
m_Grid.SetItemText(i,  6, ftos(paper.m_dDivC1*m_MainThread.m_dFactorC+paper.m_dDivX1*m_MainThread.m_dFactorX, 2));
m_Grid.SetItemText(i,  7, ftos(paper.m_dDivC *m_MainThread.m_dFactorC+paper.m_dDivX *m_MainThread.m_dFactorX, 2));

m_Grid.SetItemText(i,  8, ftos(paper.m_dDivC, 2));
m_Grid.SetItemText(i,  9, ftos(paper.m_dDivX, 2));

m_Grid.SetItemText(i, 12, itos(paper.m_nMaxQty));
m_Grid.SetItemText(i, 13, ftos(dRealPart, 2));


double m_dFactorC;      // коэффициент C
double m_dFactorX;      // коэффициент X

  CExArray<double> m_aC; // массив цен
  double m_dSumC;         // сумма цен
  double m_dMAC;          // МА цен
  double m_dLastС;        // котировка цены
  double m_dDivC;         // последняя цена/МА

  double m_dSumC1;         // сумма цен
  long   m_nNumC1;         // кол-во цен
  double m_dMAC1;          // МА цен
  double m_dLastС1;        // котировка цены
  double m_dDivC1;         // последняя цена/МА

  double m_dSumC2;         // сумма цен
  long   m_nNumC2;         // кл-во цен
  double m_dMAC2;          // МА цен
  double m_dLastС2;        // котировка цены
  double m_dDivC2;         // последняя цена/МА

  CExArray<double> m_aX; // массив X (отношений цены к индексу)
  double m_dSumX;         // сумма X
  double m_dMAX;          // МА X
  double m_dLastX;        // котировка X
  double m_dDivX;         // последняя X/МА X

  double m_dSumX1;         // сумма X
  long   m_nNumX1;         // кол-во X
  double m_dMAX1;          // МА X
  double m_dLastX1;        // котировка X
  double m_dDivX1;         // последняя X/МА X

  double m_dSumX2;         // сумма X
  long   m_nNumX2;         // кол-во X
  double m_dMAX2;          // МА X
  double m_dLastX2;        // котировка X
  double m_dDivX2;         // последняя X/МА X


  */