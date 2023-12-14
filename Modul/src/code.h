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
    printf(" Data2Driver.radius= %f ", Data2Driver_receive.radius);
    printf(" Data2Driver.speed= %f ", Data2Driver_receive.speed);
    setSpeed_time(Data2Driver_receive.speed, Data2Driver_receive.radius, 1000);
    // setSpeed_time(0.2, 0.2, 1000);
  }
  command_pred = Data2Driver_receive.startStop; // Запоминаяем команду
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

#endif