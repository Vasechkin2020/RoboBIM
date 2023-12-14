#ifndef CODE_H
#define CODE_H

// Файлы с функциями отдельных сущностей
#include "i2c_my.h"
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

float lazer_L = 0; // Данные с датчиков лазерных левого и правого
float lazer_R = 0; // Данные с датчиков лазерных левого и правого


#include "uzi.h"

// Настройка пинов светодиодов
void initLed()
{
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_ANALIZ, OUTPUT);

  digitalWrite(PIN_LED_GREEN, 1);
}

// Функция мигания светодиодом в основном цикле
void Led_Blink(int led_, unsigned long time_)
{
  static unsigned long led_time = 0;
  static bool led_status = 0;
  if ((millis() - led_time) > time_)
  {
    led_status = 1 - led_status;
    digitalWrite(led_, led_status);
    // digitalWrite(PIN_POWER_OFF, 1); // Выключение питания
    // delay(50);
    // digitalWrite(PIN_POWER_OFF, 0); // Выключение питания
    led_time = millis();
  }
}

void blink_led()
{
  if (flag_setup == 0)
  {
    Led_Blink(PIN_LED_GREEN, 500); // Мигаем светодиодом каждую 1/2 секунду, что-бы было видно что цикл не завис
  }
  else
  {
    Led_Blink(PIN_LED_GREEN, 100); // Мигаем светодиодом каждую 1/2 секунду, что-бы было видно что цикл не завис
  }
  if (millis() > 10000 && ff)
  {
    stopMotor();
    ff = false;
  }
}

// Печать пришедших данных
void printData2Driver_receive()
{
  printf(" id= %i ", Data2Driver_receive.id);
  printf(" startStop= %i ", Data2Driver_receive.control.startStop);
  printf(" speed= %f ", Data2Driver_receive.control.speed);
  printf(" radius= %f ", Data2Driver_receive.control.radius);
  printf(" command1= %i ", Data2Driver_receive.control.command1);
  printf(" command2= %i ", Data2Driver_receive.control.command2);
  printf(" servo1.position= %i ", Data2Driver_receive.servo1.position);
  printf(" servo1.time= %i ", Data2Driver_receive.servo1.time);
  printf(" servo2.position= %i ", Data2Driver_receive.servo2.position);
  printf(" servo2.time= %i ", Data2Driver_receive.servo2.time);
  printf(" led.num_program= %i ", Data2Driver_receive.led.num_program);
  printf(" \n ");
}

// Отработка пришедших команд. Изменение скорости, траектории и прочее
void executeCommand()
{
// Управление шаговыми мотрами движения
#ifdef MOTOR
  static int command_pred = 0;                                                                             // Переменная для запоминания предыдущей команды
                                                                                                           // printf(" Data2Driver_receive.control.startStop= %i \n", Data2Driver_receive.control.startStop);
  if (Data2Driver_receive.control.startStop == 0 && Data2Driver_receive.control.startStop != command_pred) // Если пришла команда 0 и предыдущая была другая
  {
    // Serial.println("commanda  STOP...");
    stopMotor(); // все останавливаем
  }
  if (Data2Driver_receive.control.startStop == 1) // Если команда двигаться то задаем движение на 1 секунду
  {
    // printf(" Data2Driver.radius= %f ", Data2Driver_receive.radius);
    // printf(" Data2Driver.speed= %f \n", Data2Driver_receive.speed);
    setSpeed_time(Data2Driver_receive.control.speed, Data2Driver_receive.control.radius, 3000);
    // setSpeed_time(0.2, 0.2, 1000);
  }
  command_pred = Data2Driver_receive.control.startStop; // Запоминаем команду
#endif

// Управление сервомоторами рук
#ifdef MOTOR_SERVO_def
  // servo1.position = getPosServo_L(); // Считываем положение мотора
  // servo2.position = getPosServo_R(); // Считываем положение мотора
  // if (Data2Driver_receive.servo1.position == 0)
  // {
  //   runServo_0(1000); // В начальное положение
  // }
  // else
  // {
  //   runServo_1(1000); // Машем вверх и вниз
  // }
#endif

// Управление светодиодами
#ifdef LED_def
  switch (Data2Driver_receive.led)
  {
  case 0:
    if (led2812.flag_led_off == 0)
      led2812.run0();
    break;
  case 1:
    led2812.run1(); // Основной цикл мигания светодиодами. В нем пишем все чем захочем мигать. Занимает примерно 1,5 миллисекунды
    break;
  case 2:
    led2812.run2();
    break;
  }
#endif
}

// Функция исполняемая по прерыванию по таймеру 0
void IRAM_ATTR onTimer() // Обработчик прерывания таймера 0 по совпадению A 	1 раз в 1 милисекунду
{
  count_timer_10millisec++;
  count_timer_50millisec++;
  count_timer_1sec++;
  count_timer_60sec++;
  // Serial.println(".");
  //  каждые 10 милисекунд
  if (count_timer_10millisec >= 10)
  {
    count_timer_10millisec = 0;
    flag_timer_10millisec = true;
  }
  // каждые 50 милисекунд
  if (count_timer_50millisec >= 50)
  {
    count_timer_50millisec = 0;
    flag_timer_50millisec = true;
  }
  // 1 seconds
  if (count_timer_1sec >= 1000)
  {
    count_timer_1sec = 0;
    flag_timer_1sec = true;
  }
  // Таймер на 1 минуту
  if (count_timer_60sec >= 60000)
  {
    count_timer_60sec = 0;
    flag_timer_60sec = true;
  }
}

// Инициализация таймера 0. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void initTimer_0()
{
  printf("initTimer_0 \n");
  timer0 = timerBegin(0, 80, true);             // Номер таймера, делитель(прескаллер), Считать вверх, прибавлять (true)  Частота базового сигнала 80  Мега герц, значит будет 1 микросекунда
  timerAttachInterrupt(timer0, &onTimer, true); // Какой таймер используем, какую функцию вызываем,  тип прерывания  edge или level interrupts
  timerAlarmWrite(timer0, 1000, true);          // Какой таймер, до скольки считаем , сбрасываем ли счетчик при срабатывании 1000 микросекунд эти 1 милисекунда
  timerAlarmEnable(timer0);                     // Запускаем таймер
}

// Собираем нужные данные и пишем в структуру на отправку
void collect_Driver2Data()
{
  Driver2Data_send.id++;

  Driver2Data_send.status.timeStart = status.timeStart;
  Driver2Data_send.status.countCommand = status.countCommand;
  Driver2Data_send.status.countBedCommand = status.countBedCommand;

  Driver2Data_send.car.radiusSet = car.radiusSet;
  Driver2Data_send.car.radiusEncod = car.radiusEncod;
  Driver2Data_send.car.speedSet = car.speedSet;
  Driver2Data_send.car.speedEncod = car.speedEncod;
  Driver2Data_send.car.way = car.way;

  Driver2Data_send.motorLeft.way = motorLeft.way;
  Driver2Data_send.motorLeft.rpsSet = motorLeft.rpsSet;
  Driver2Data_send.motorLeft.rpsEncod = motorLeft.rpsEncod;

  Driver2Data_send.motorRight.way = motorRight.way;
  Driver2Data_send.motorRight.rpsSet = motorRight.rpsSet;
  Driver2Data_send.motorRight.rpsEncod = motorRight.rpsEncod;

  Driver2Data_send.odom_enc = odom_enc;
  Driver2Data_send.bno055 = bno055;

  Driver2Data_send.servo1.position = servo1.position;
  Driver2Data_send.servo2.position = servo2.position;

  Driver2Data_send.lazer1.distance = lazer1.distance;
  Driver2Data_send.lazer2.distance = lazer2.distance;
  Driver2Data_send.uzi1.distance = uzi1.distance;
  Driver2Data_send.uzi2.distance = uzi2.distance;

  Driver2Data_send.cheksum = measureCheksum(Driver2Data_send); // Вычисляем контрольную сумму структуры и пишем ее значение в последний элемент

  // Serial.println("");
}
void printBody()
{

  printf(" id= %i \n", Driver2Data_send.id);
  // printf(" odom_L= %f \n", Driver2Data_send.odom_L);
  // printf(" odom_R= %f \n", Driver2Data_send.odom_R);
  // printf(" speed_L= %f \n", Driver2Data_send.speed_L);
  // printf(" speed_R= %f \n", Driver2Data_send.speed_R);
  // printf(" roll= %f \n", Driver2Data_send.bno055.roll);
  // printf(" pitch= %f \n", Driver2Data_send.bno055.pitch);
  // printf(" yaw= %f \n", Driver2Data_send.bno055.yaw);
  // printf(" voltage= %f \n", Driver2Data_send.ina.voltage);
  // printf(" current= %f \n", Driver2Data_send.ina.current);
  // printf(" capacity_percent= %f \n", Driver2Data_send.ina.capacity_percent);
  // printf(" capacity_real= %f \n", Driver2Data_send.ina.capacity_real);
  printf(" Send cheksum= %i  \n --- \n", Driver2Data_send.cheksum);
}

uint32_t Read_VL53L0X(VL53L0X &sensor_, byte line_)
{
	set_TCA9548A(line_);
  uint32_t rez = 0;
  // float rez = sensor.readRangeContinuousMillimeters();
  if ((sensor_.readReg(0x13) & 0x07) != 0)
  {
    rez = sensor_.readReg16Bit(0x1E);
    sensor_.writeReg(0x0B, 0x01);
  }
  else
  {
    Serial.println("Error Read_VL53L0X.");
  }
  return rez;
}
// Инициализация левого датчика с адресом 0x30
void Init_VL53L0X(VL53L0X &sensor_, byte adr, byte line_)
{
  Serial.println(" ======================================== Init_VL53L0X ===================================");

  set_TCA9548A(line_);
  sensor_.setTimeout(1000); // Время на выполения функций конфигурации если превышено выдает ошибку
  sensor_.newAddress(adr);

  if (sensor_.init())
  {
    Serial.println("Init_VL53L0X OK !!!");
    // Start continuous back-to-back mode (take readings as	 fast as possible).  To use continuous timed mode  instead, provide a desired inter-measurement period in  ms (e.g. sensor.startContinuous(100)).
    sensor_.startContinuous();
    sensor_.setMeasurementTimingBudget(25000);
  }
  else
  {
    Serial.print(" Линия ");
    Serial.print(line_);
    Serial.print(" . Не нашли на адресе");
    Serial.print(adr, HEX);
    Serial.println(" !!!!!!!!!!!!!!!!!! Failed to detect and initialize sensor!");
    delay(2000);
  }
  byte address = sensor_.readReg(0x8A);
  Serial.print(" address = ");
  Serial.print(address, HEX);
  byte WiA = sensor_.readReg(0xC0);
  Serial.print(" WiA = ");
  Serial.println(WiA);

  // uint8_t address_I2C = sensor_.getAddress();
  // printf("Adress i2c= %i ", address_I2C);
  // delay(1000);

  Serial.println("End Init_VL53L0X ");
}

void loop_VL53L0X()
{

  float lazer_L_temp = Read_VL53L0X(Sensor_VL53L0X_L, multi_line_VL53L0X_L) / 1000.0; // Преобразуем в метры
  float lazer_R_temp = Read_VL53L0X(Sensor_VL53L0X_R, multi_line_VL53L0X_R) / 1000.0; // Преобразуем в метры

  // lazer_L = (lazer_L_temp * 0.66) + (lazer_L * 0.34); // Небольшое сглаживание с прошлым результатом Можно отфильтровать Калманом если нужно
  // lazer_R = (lazer_R_temp * 0.66) + (lazer_R * 0.34); // Небольшое сглаживание с прошлым результатом Можно отфильтровать Калманом если нужно

  lazer_L = filtr_My(lazer_L, lazer_L_temp, 0.90);
  lazer_R = filtr_My(lazer_R, lazer_R_temp, 0.90);

  // lazer_L = lazer_L_temp;
  // lazer_R = lazer_R_temp;

  // Ограничение в 1 метр
  if (lazer_L > 1)
    lazer_L = 1.111;
  if (lazer_R > 1)
    lazer_R = 1.111;

  lazer1.distance = lazer_L;
  lazer2.distance = lazer_R;

  // Serial.printf(" lazer1.distance = %f ", lazer1.distance);
  // Serial.printf(" lazer2.distance = %f \n", lazer2.distance);

}

void min_1()
{
  //----------------------------- 1 МИНУТА !!!!  --------------------------------------
  if (flag_timer_60sec) // Вызывается каждую МИНУТУ
  {
    flag_timer_60sec = false;

    // Печать времени что программа не зависла, закомментировать в реальной работе
    // printf(" %f \n", millis() / 1000.0);
  }
  //***********************************************************************
}

void sec_1()
{
  //----------------------------- 1 СЕКУНДА !!!!  --------------------------------------
  if (flag_timer_1sec) // Вызывается каждую секунду
  {
    flag_timer_1sec = false;
    // setSpeed_time(0.2, 0.5, 500);
    //  ppp += 0.1;
    //  printf(" ppp= %f \n", ppp);
    //  setSpeedRPS_R(ppp);
    //  setSpeedMS_R(2);
    // int pos = SERVO.GetPos(ID_SERVO_LP);
    // Serial.print(" posL = ");
    // Serial.print(pos);
    // pos = SERVO.GetPos(ID_SERVO_LP);
    // Serial.print(" posR = ");
    // Serial.print(pos);
    printf(" %f \n", millis() / 1000.0);

    // Serial.print(" Data2Driver_receive.led.num_program = ");
    // Serial.println(Data2Driver_receive.led.num_program);

#ifdef RCWL1601_def
    // Serial.print(" distance = ");
    // Serial.print(distance_uzi);
#endif

#ifdef VL530L0X_def
    // Serial.print(" distance lazerL= ");
    // Serial.print(lazer_L);
    // Serial.print(" distance lazerR= ");
    // Serial.println(lazer_R);
#endif
    // printRemoteXY();
    // printBody();
  }
}

void millisec_50()
{
  //----------------------------- 50 миллисекунд --------------------------------------
  if (flag_timer_50millisec)
  {
    flag_timer_50millisec = false;

    if (flag_newData || flag_newControlData) // Выполняем если есть новые данные или по шине или вручную если не будет команд ни по шине не вручную то робот остановиться
    {
      executeCommand();        // Выполнение пришедших команд
      flag_newData = 0;        // Сброс флага
      flag_newControlData = 0; // Сброс флага
    }

#ifdef MOTOR             // Контроль отключения драйверов моторов после остановки
    calculateOdom_enc(); // Подсчет одометрии по энкодерам

    // setSpeed_time(0.2, 0.2, 1000);
    // Serial.println("setSpeed_time");
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
}

void millisec_10()
{
  //----------------------------- 10 миллисекунд --------------------------------------
  if (flag_timer_10millisec)
  {
    flag_timer_10millisec = false;
    // servoTest();
    getKoefAccel();      // Расчет коэффициента ускорений при движении по радиусу
    setAcceleration_L(); // Контроль за ускорением мотора
    setAcceleration_R(); // Контроль за ускорением мотора

#ifdef MOTOR        // Контроль за моторами по условиям
    movementTime(); // Отслеживание времени движения
    delayMotor();   // Задержка отключения драйверов моторов после остановки
#endif
#ifdef REMOTEXY
    if (RemoteXY.connect_flag == 1) // Если есть связь то берем данные от ручного управления и заменяем те что пришли по шине
    {
      changeDataFromRemoteXY(); // Заменяем данные, если мы на ручном управлении
    }
#endif
    // Serial.println(String(millis()) + " flag_newData = " + flag_newData);
    // Serial.println(String(millis()) + " flag_newControlData = "+ flag_newControlData);

#ifdef BNO_def
    BNO055_readEuler();  // Опрашиваем датчик получаем углы
    BNO055_readLinear(); // Опрашиваем датчик получаем ускорения
    calculateOdom_imu();    // Подсчет одометрии по imu
#endif

#ifdef RCWL1601_def
    loopUzi(); // Все действия по ультразвуковому датчику

    // inTimerUzi(); // Проверка на случай если сигнал не вернется тогда через заданное время сбросим (было так раньше, потом перенес в основную функцию loopUzi())
#endif
  }
}

#endif