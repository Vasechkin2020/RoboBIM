#ifndef CODE_H
#define CODE_H

#include "VL53L0X.h" // Чужая библиотека polulu https://github.com/pololu/vl53l0x-arduino

//************************ ОБЬЯВЛЕНИЕ ФУНКЦИЙ *******************************************

void set_TCA9548A(uint8_t bus);                                 // Функция устанавляивающая нужное положение на мультиплексоре
void initLed();                                                 // Настройка пинов светодиодов
void Led_Blink(int led_, unsigned long time_);                  // Функция мигания светодиодом в основном цикле
void ledBlink(int ledStart_, int ledEnd_, unsigned long time_); // Функция мигания группой светодиодов в основном цикле
void printData2Driver_receive();                                // Печать пришедших данных
void executeCommand();                                          // Отработка пришедших команд. Изменение скорости, траектории и прочее
void IRAM_ATTR onTimer();                                       // Обработчик прерывания таймера 0 по совпадению A 	1 раз в 1 милисекунду // Функция исполняемая по прерыванию по таймеру 0
void initTimer_0();                                             // Инициализация таймера 0. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void collect_Driver2Data();                                     // Собираем нужные данные и пишем в структуру на отправку
void printBody();                                               //
SSensor Read_VL53L0X(VL53L0X &sensor_, byte line_);             //
void Init_VL53L0X(VL53L0X &sensor_, byte adr, byte line_);      // Инициализация левого датчика с адресом 0x30
void loop_VL53L0X();                                            //
//***************************************************************************************

// Файлы с функциями отдельных сущностей
#include "i2c_my.h"
#include "motor.h"
#include "protokolSPI.h"
#include "bno.h"

#include "led43.h"
led43 led2812; // Экземпляр класса

VL53L0X Sensor_VL53L0X_L;
VL53L0X Sensor_VL53L0X_R;

float laser_L = 0; // Данные с датчиков лазерных левого и правого
float laser_R = 0; // Данные с датчиков лазерных левого и правого

#include "uzi.h"

// Функция устанавляивающая нужное положение на мультиплексоре
void set_TCA9548A(uint8_t bus_)
{
  if (bus_ > 7)
    return;
  // Serial.printf("set_TCA9548A -> %i \n", bus_);
  Wire.beginTransmission(Addr_TCA9548A); // TCA9548A адрес 0x70  or 0x77
  Wire.write(1 << bus_);                 // отправляем байт на выбранную шину
  Wire.endTransmission();
}
// set_TCA9548A(num_line_); // Переключаем мультиплексор

// Настройка пинов светодиодов
void initLed()
{
  pinMode(PIN_LED_GREEN, OUTPUT);
  // pinMode(PIN_ANALIZ, OUTPUT);

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

// Функция мигания группой светодиодов в основном цикле
void ledBlink(int ledStart_, int ledEnd_, unsigned long time_)
{
  static unsigned long led_time = 0;
  static bool led_status = 0;
  if ((millis() - led_time) > time_)
  {
    led_status = 1 - led_status;
    if (led_status == 1)
    {
      led2812._ledUp(ledStart_, ledEnd_, green);
    }
    else
    {
      led2812._ledUp(ledStart_, ledEnd_, black);
    }
    led_time = millis();
  }
}

// Печать пришедших данных
void printData2Driver_receive()
{
  printf(" id= %i ", Data2Driver_receive.id);
  printf(" speedL= %f ", Data2Driver_receive.control.speedL);
  printf(" speedR= %f ", Data2Driver_receive.control.speedR);
  // printf(" led.num_program= %i ", Data2Driver_receive.led.num_program);
  printf(" \n ");
}

// Отработка пришедших команд. Изменение скорости, траектории и прочее
void executeCommand()
{
  static SControl predControl;
// Управление шаговыми мотрами движения
#ifdef MOTOR
  if (Data2Driver_receive.control.speedL != 0 || Data2Driver_receive.control.speedR != 0 || predControl.speedL != 0 || predControl.speedR != 0) // Если хоть одна скорость не равна нулю то исполняем, иначе пропускаем и драйвера отключаться.
  {
    flagExecuteCommand = true;
    timeExecuteCommand = millis();                  // Запоминаем время когда дали команду моторам вращаться. Если через 1 секунду не поступит новой команды моторы остановятся сами movementTime()
    setSpeed_L(Data2Driver_receive.control.speedL); // Задаем скорость левого колеса в оборотах в секунду
    setSpeed_R(Data2Driver_receive.control.speedR); // Задаем скорость правого колеса в оборотах в секунду
  }
  predControl = Data2Driver_receive.control; // Запоминаем на следующий раз какая была ранее задана скорость
#endif

// Управление светодиодами
#ifdef LED_def
  led2812._translate(Data2Driver_receive.led); // Передача полученных данных по светодиодам
#endif
}

// Функция исполняемая по прерыванию по таймеру 0
void IRAM_ATTR onTimer() // Обработчик прерывания таймера 0 по совпадению A 	1 раз в 1 милисекунду
{
  count_timer_10millisec++;
  count_timer_50millisec++;
  count_timer_1sec++;
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
  Driver2Data_send.motor = motor;
  // Driver2Data_send.bno055 = bno055;
  Driver2Data_send.laserL = laserL;
  Driver2Data_send.laserR = laserR;
  Driver2Data_send.uzi = uzi;
  Driver2Data_send.spi = spi;
  Driver2Data_send.cheksum = measureCheksum(Driver2Data_send); // Вычисляем контрольную сумму структуры и пишем ее значение в последний элемент
}

//
void printBody()
{
  printf(" id= %i \n", Driver2Data_send.id);
  // printf(" capacity_real= %f \n", Driver2Data_send.ina.capacity_real);
  printf(" Send cheksum= %i  \n --- \n", Driver2Data_send.cheksum);
}
//
SSensor Read_VL53L0X(VL53L0X &sensor_, byte line_)
{
  set_TCA9548A(line_);
  SSensor rez;
  // float rez = sensor.readRangeContinuousMillimeters();
  if ((sensor_.readReg(0x13) & 0x07) != 0)
  {
    rez.distance = sensor_.readReg16Bit(0x1E);
    rez.status = 1; // Статус все хорошо
    sensor_.writeReg(0x0B, 0x01);
  }
  else
  {
    rez.status = -1; // Статус все плохо
    // Serial.print("Error Read_VL53L0X. Line= ");
    // Serial.println(line_);
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
// Считывание датчиков, фильтрация и установка статуса
void loop_VL53L0X()
{

  SSensor laser_L_temp = Read_VL53L0X(Sensor_VL53L0X_L, multi_line_VL53L0X_L); // Считываем значение и статус
  SSensor laser_R_temp = Read_VL53L0X(Sensor_VL53L0X_R, multi_line_VL53L0X_R); // Считываем значение и статус

  float distanceL = laser_L_temp.distance / 1000.0; // Преобразуем в метры
  float distanceR = laser_R_temp.distance / 1000.0; // Преобразуем в метры
  // laser_L = (laser_L_temp * 0.66) + (laser_L * 0.34); // Небольшое сглаживание с прошлым результатом Можно отфильтровать Калманом если нужно
  // laser_R = (laser_R_temp * 0.66) + (laser_R * 0.34); // Небольшое сглаживание с прошлым результатом Можно отфильтровать Калманом если нужно

  laser_L = filtr_My(laser_L, distanceL, 0.90); // Небольшое сглаживание с прошлым результатом Берем предыдущее значение и немного нового
  laser_R = filtr_My(laser_R, distanceR, 0.90);

  // laser_L = laser_L_temp;
  // laser_R = laser_R_temp;

  // Ограничение в 1 метр
  if (laser_L > 1)
    laser_L = 1.111;
  if (laser_R > 1)
    laser_R = 1.111;

  laserL.distance = laser_L;
  laserL.status = laser_L_temp.status;

  laserR.distance = laser_R;
  laserR.status = laser_R_temp.status;

  //   Serial.printf(" laserL.distance = %f ", laserL.distance);
  //   Serial.printf(" laserR.distance = %f \n", laserR.distance);
}

// Функция выравнивания верхней платформы
void platforma()
{
  int l = 0;
  int r = 0;
  if (BNO055_EulerAngles.y > 0.1 && BNO055_EulerAngles.y < 3) // Управление вверх нос или вниз pitch
  {
    l += 1;
    r += 1;
  }
  if (BNO055_EulerAngles.y < -0.1 && BNO055_EulerAngles.y > -3)
  {
    l -= 1;
    r -= 1;
  }
  if (BNO055_EulerAngles.y >= -0.1 && BNO055_EulerAngles.y <= 0.1)
  {
    l = r = 0;
  }

  if (BNO055_EulerAngles.x > 0.1 && BNO055_EulerAngles.x < 3) // Управление наклон вправо влево roll
  {
    l -= 1;
    r += 1;
  }
  if (BNO055_EulerAngles.x < -0.1 && BNO055_EulerAngles.x > -3)
  {
    l += 1;
    r -= 1;
  }
  if (BNO055_EulerAngles.x >= -0.1 && BNO055_EulerAngles.x <= 0.1)
  {
    // l += 1;
    // r += 1;
  }

  if (l > 0)
  {
    digitalWrite(Pla_PIN_En, 0);    // 0- Разрешена работа 1- запрещена работа драйвера
    digitalWrite(Pla_PIN_L_Dir, 1); //
    flag_motor_PLA_L = 1;           // Разрешение на импульсы
  }
  else
  {
    if (l < 0)
    {
      digitalWrite(Pla_PIN_En, 0);    // 0- Разрешена работа 1- запрещена работа драйвера
      digitalWrite(Pla_PIN_L_Dir, 0); //
      flag_motor_PLA_L = 1;           // Разрешение на импульсы
    }
    if (l == 0)
    {
      flag_motor_PLA_L = 0; // Запрет на импульсы
    }
  }

  if (r > 0)
  {
    digitalWrite(Pla_PIN_En, 0);    // 0- Разрешена работа 1- запрещена работа драйвера
    digitalWrite(Pla_PIN_R_Dir, 1); //
    flag_motor_PLA_R = 1;           // Разрешение на импульсы
  }
  else
  {
    if (r < 0)
    {
      digitalWrite(Pla_PIN_En, 0);    // 0- Разрешена работа 1- запрещена работа драйвера
      digitalWrite(Pla_PIN_R_Dir, 0); //
      flag_motor_PLA_R = 1;           // Разрешение на импульсы
    }
    if (r == 0)
    {
      flag_motor_PLA_R = 0; // Запрет на импульсы
    }
  }
  if (l == 0 && r == 0)
  {
    digitalWrite(Pla_PIN_En, 1); // 0- Разрешена работа 1- запрещена работа драйвера
  }
  printf("l= %i r= %i \n", l, r);
}
#endif