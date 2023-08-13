#ifndef CODE_H
#define CODE_H

// Функция фильтрующая(сглаживающая) значения берет старое с меньшим весом и новое с большим
float filtr_My(float old_, float new_, float ves_new_)
{
  return (old_ * (1.0 - ves_new_)) + (new_ * ves_new_);
}

void initLed()
{
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_ANALIZ, OUTPUT);

  digitalWrite(PIN_LED_GREEN, 1);
}

void offLed()
{
  digitalWrite(PIN_LED_GREEN, 0);
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

// Отработка пришедших команд. Изменение скорости, траектории и прочее
void executeCommand()
{
// Управление шаговыми мотрами движения
#ifdef MOTOR
  static int command_pred = 0;                                                             // Переменная для запоминания предыдущей команды
  if (Data2Driver_receive.startStop == 0 && Data2Driver_receive.startStop != command_pred) // Если пришла команда 0 и предыдущая была другая
  {
    // Serial.println("commanda  STOP...");
    stopMotor(); // все останавливаем
  }
  if (Data2Driver_receive.startStop == 1) // Если командв двигаться то задаем движение на 1 секунду
  {
    // printf(" Data2Driver.radius= %f ", Data2Driver_receive.radius);
    // printf(" Data2Driver.speed= %f \n", Data2Driver_receive.speed);
    setSpeed_time(Data2Driver_receive.speed, Data2Driver_receive.radius, 1000);
    // setSpeed_time(0.2, 0.2, 1000);
  }
  command_pred = Data2Driver_receive.startStop; // Запоминаем команду
#endif

// Управление сервомоторами рук
#ifdef MOTOR_SERVO_def
  g_posServoL = getPosServo_L(); // Считываем положение мотора
  g_posServoR = getPosServo_R(); // Считываем положение мотора
  if (Data2Driver_receive.servo == 0)
  {
    runServo_0(1000); // В начальное положение
  }
  else
  {
    runServo_1(1000); // Машем вверх и вниз
  }
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
  timer0 = timerBegin(0, 80, true);             // Номер таймера, делитель(прескаллер), Считать вверх, прибавлять (true)  Частота базового сигнала 80  Мега герц, значит будет 1 микросекунда
  timerAttachInterrupt(timer0, &onTimer, true); // Какой таймер используем, какую функцию вызываем,  тип прерывания  edge или level interrupts
  timerAlarmWrite(timer0, 1000, true);          // Какой таймер, до скольки считаем , сбрасываем ли счетчик при срабатывании 1000 микросекунд эти 1 милисекунда
  timerAlarmEnable(timer0);                     // Запускаем таймер
}

// Собираем нужные данные и пишем в структуру на отправку
void collect_Data()
{
  Driver2Data_send.id++;
  Driver2Data_send.odom_enc = g_odom_enc;
  Driver2Data_send.odom_imu = g_odom_imu;
  Driver2Data_send.odom_L = odom_way_L;
  Driver2Data_send.odom_R = odom_way_R;
  Driver2Data_send.speed_L = speed_L_fact;
  Driver2Data_send.speed_R = speed_R_fact;
  Driver2Data_send.bno055.roll = BNO055_EulerAngles.x;
  Driver2Data_send.bno055.pitch = BNO055_EulerAngles.y;
  Driver2Data_send.bno055.yaw = BNO055_EulerAngles.z;

  Driver2Data_send.connect_flag = RemoteXY.connect_flag;
  Driver2Data_send.startStop = RemoteXY.startStop;
  Driver2Data_send.radius = RemoteXY.radius;
  Driver2Data_send.servo = RemoteXY.servo;
  Driver2Data_send.led = RemoteXY.led;
  Driver2Data_send.angle_camera = RemoteXY.camera;
  Driver2Data_send.status_wifi = flag_wifi;
  Driver2Data_send.posServoL = g_posServoL;
  Driver2Data_send.posServoR = g_posServoL;

  if (RemoteXY.connect_flag == 1) // Если есть связь то берем данные от ручного управления и заменяем те что пришли по шине
  {
    Driver2Data_send.speed = FIX_SPEED; // Скорость идет особо. Возвращаем ту которую пришло от Data  или ту которую поменили в ручном варианте на постоянную
  }
  else
  {
    Driver2Data_send.speed = Data2Driver_receive.speed; // Скорость идет особо. Возвращаем ту которую пришло от Data  или ту которую поменили в ручном варианте на постоянную
  }

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

float lazer_L = 0; // Данные с датчиков лазерных левого и правого
float lazer_R = 0; // Данные с датчиков лазерных левого и правого

uint32_t Read_VL53L0X(VL53L0X &sensor_)
{
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
void Init_VL53L0X(VL53L0X &sensor_, byte adr)
{
  Serial.println("-------------------------------------------------------------");
  Serial.println("Start Init_VL53L0X ...");

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
    Serial.print(" Не нашли на адресе "); 
    Serial.print(adr, HEX);
    Serial.print( ". Пробуем установить 0x29 и заменить на ");
    Serial.println(adr, HEX);

    sensor_.newAddress(0x29);
    sensor_.setAddress(adr);
    if (sensor_.init())
    {
      Serial.println("Init_VL53L0X OK !!!");
      sensor_.startContinuous();
      sensor_.setMeasurementTimingBudget(25000);
    }
    else
    {
      Serial.println(" !!!!!!!!!!!!!!!!!! Failed to detect and initialize sensor!");
      delay(9999);
    }
  }
  byte address = sensor_.readReg(0x8A);
  Serial.print(" address = ");
  Serial.print(address, HEX);
  byte WiA = sensor_.readReg(0xC0);
  Serial.print(" WiA = ");
  Serial.println(WiA);

  // uint8_t address_I2C = sensor_.getAddress();
  // printf("Adress i2c= %i ", address_I2C);
  delay(1000);

  Serial.println("End Init_VL53L0X Left.");
  Serial.println("-------------------------------------------------------------");
}

void loop_VL53L0X()
{

  float lazer_L_temp = Read_VL53L0X(Sensor_VL53L0X_L) / 1000.0; // Преобразуем в метры
  float lazer_R_temp = Read_VL53L0X(Sensor_VL53L0X_R) / 1000.0; // Преобразуем в метры

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

  Serial.print(" Sensor_VL53L0X_L = ");
  Serial.print(lazer_L, 3);
  Serial.print(" Sensor_VL53L0X_R = ");
  Serial.print(lazer_R, 3);
  Serial.println("");
}

#endif