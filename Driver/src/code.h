#ifndef CODE_H
#define CODE_H

#include "VL53L0X.h" // Чужая библиотека polulu https://github.com/pololu/vl53l0x-arduino

//************************ ОБЬЯВЛЕНИЕ ФУНКЦИЙ *******************************************

void set_TCA9548A(uint8_t bus);                // Функция устанавляивающая нужное положение на мультиплексоре
void initLed();                                // Настройка пинов светодиодов
void Led_Blink(int led_, unsigned long time_); // Функция мигания светодиодом в основном цикле
void printData2Driver_receive();                           // Печать пришедших данных
void executeCommand();                                     // Отработка пришедших команд. Изменение скорости, траектории и прочее
void IRAM_ATTR onTimer();                                  // Обработчик прерывания таймера 0 по совпадению A 	1 раз в 1 милисекунду // Функция исполняемая по прерыванию по таймеру 0
void initTimer_0();                                        // Инициализация таймера 0. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void collect_Driver2Data();                                // Собираем нужные данные и пишем в структуру на отправку
void printBody();                                          //
uint32_t Read_VL53L0X(VL53L0X &sensor_, byte line_);       //
void Init_VL53L0X(VL53L0X &sensor_, byte adr, byte line_); // Инициализация левого датчика с адресом 0x30
void loop_VL53L0X();                                       //
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

float lazer_L = 0; // Данные с датчиков лазерных левого и правого
float lazer_R = 0; // Данные с датчиков лазерных левого и правого

#include "uzi.h"

// Функция устанавляивающая нужное положение на мультиплексоре
void set_TCA9548A(uint8_t bus)
{
  if (bus > 7)
    return;
  Wire.beginTransmission(Addr_TCA9548A); // TCA9548A адрес 0x70  or 0x77
  Wire.write(1 << bus);                  // отправляем байт на выбранную шину
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

// Печать пришедших данных
void printData2Driver_receive()
{
  printf(" id= %i ", Data2Driver_receive.id);
  printf(" speedL= %f ", Data2Driver_receive.control.speedL);
  printf(" speedR= %f ", Data2Driver_receive.control.speedR);
  printf(" led.num_program= %i ", Data2Driver_receive.led.num_program);
  printf(" \n ");
}

// Отработка пришедших команд. Изменение скорости, траектории и прочее
void executeCommand()
{
// Управление шаговыми мотрами движения
#ifdef MOTOR
  flagExecuteCommand = true;
  timeExecuteCommand = millis();   // Запоминаем время когда дали команду моторам вращаться. Если через 1 секунду не поступит новой команды моторы остановятся сами movementTime()
  setSpeed_L(Data2Driver_receive.control.speedL); // Задаем скорость левого колеса в метрах в секунду
  setSpeed_R(Data2Driver_receive.control.speedR); // Задаем скорость правого колеса в метрах в секунду
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


  Driver2Data_send.car.speedEncod = car.speedEncod;

  Driver2Data_send.motorLeft.rpsSet = motorLeft.rpsSet;
  Driver2Data_send.motorLeft.rpsEncod = motorLeft.rpsEncod;

  Driver2Data_send.motorRight.rpsSet = motorRight.rpsSet;
  Driver2Data_send.motorRight.rpsEncod = motorRight.rpsEncod;

  Driver2Data_send.bno055 = bno055;

  Driver2Data_send.lazer1.distance = lazer1.distance;
  Driver2Data_send.lazer2.distance = lazer2.distance;
  Driver2Data_send.uzi1.distance = uzi.distance;
  
  Driver2Data_send.spi.all = spi.all;
  Driver2Data_send.spi.bed = spi.bed;

  Driver2Data_send.cheksum = measureCheksum(Driver2Data_send); // Вычисляем контрольную сумму структуры и пишем ее значение в последний элемент

  // Serial.println("");
}

//
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
//
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

#endif