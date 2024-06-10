#ifndef CODE_H
#define CODE_H

// Функция фильтрующая(сглаживающая) значения берет старое с меньшим весом и новое с большим
float filtr_My(float old_, float new_, float ves_new_)
{
  return (old_ * (1.0 - ves_new_)) + (new_ * ves_new_);
}

//Функция записи строки в мой массив символов и обрезание строки
void stroka2Massiv(const char *stroka_, char *massiv_, byte count_)
{
  for (byte i = 0; i < count_; i++)
  {
    massiv_[i] = stroka_[i];
    if (massiv_[i] == 0) // Если закончилась строка, строка заканчивается нулем
      return;
    // printf("%c ", massiv_[i]);
  }
}

void initLed()
{
  pinMode(PIN_LED_RED, OUTPUT); //Не работает пин. При инициализации SPI_slave функция его занимает и дает всегда половину напряжения и он не реагирует на команды.
  pinMode(PIN_LED_BLUE, OUTPUT);
  digitalWrite(PIN_LED_RED, 1);  // Запустился SetUp
  digitalWrite(PIN_LED_BLUE, 1); // Запустился SetUp
}

void offLed()
{
  digitalWrite(PIN_LED_RED, 0);  // Запустился SetUp
  digitalWrite(PIN_LED_BLUE, 0); // Запустился SetUp
}
//Функция мигания светодиодом в основном цикле
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
  static int command_pred = 0; // Переменная для запоминания предыдущей команды
  // if (Data2Iot_receive.command_body == 0 && Data2Iot_receive.command_body != command_pred) // Если пришла команда 0 и предыдущая была другая
  // {
  //   // Serial.println("commanda  STOP...");
  //   // stopMotor(); //все останавливаем
  // }
  // if (Data2Iot_receive.command_body == 1) // Если командв двигаться то азадем движение на 1 секунду
  // {
  //   // setSpeed_time(Data2Iot_receive.speed, Data2Iot_receive.radius, 1000);
  //   // setSpeed_time(0.2, 0.2, 1000);
  // }
  // command_pred = Data2Iot_receive.command_body; // Запоминаяем команду
}


//Функция исполняемая по прерыванию по таймеру 0
void IRAM_ATTR onTimer() // Обработчик прерывания таймера 0 по совпадению A 	1 раз в 1 милисекунду
{
  // portENTER_CRITICAL_ISR(&timerMux);
  // interruptCounter++;
  // portEXIT_CRITICAL_ISR(&timerMux);

  count_timer_10millisec++;
  count_timer_50millisec++;
  count_timer_1sec++;
  count_timer_60sec++;

  // каждые 10 милисекунд
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

//Инициализация таймера 0. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void initTimer_0()
{
  printf(" Starrt initTimer_0 ... \n");
  timer0 = timerBegin(0, 80, true);             // Номер таймера, делитель(прескаллер), Считать вверх, прибавлять (true)  Частота базового сигнала 80  Мега герц, значит будет 1 микросекунда
  timerAttachInterrupt(timer0, &onTimer, true); // Какой таймер используем, какую функцию вызываем,  тип прерывания  edge или level interrupts
  timerAlarmWrite(timer0, 1000, true);          // Какой таймер, до скольки считаем , сбрасываем ли счетчик при срабатывании 1000 микросекунд эти 1 милисекунда
  timerAlarmEnable(timer0);                     // Запускаем таймер
}

// Собираем нужные данные и пишем в структуру на отправку
void collect_Data()
{
  Iot2Data_send.id++;

  Iot2Data_send.cheksum = measureCheksum(Iot2Data_send); // Вычисляем контрольную сумму структуры и пишем ее значение в последний элемент

  // Serial.print(" Iot2Data_send.cheksum = ");
  // Serial.println(Iot2Data_send.cheksum);
}
void printBody()
{

  printf(" id= %i \n", Iot2Data_send.id);
  printf(" Send cheksum= %i  \n --- \n", Iot2Data_send.cheksum);
}

#endif