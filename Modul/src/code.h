#ifndef CODE_H
#define CODE_H

//************************ ОБЬЯВЛЕНИЕ ФУНКЦИЙ *******************************************

void IRAM_ATTR onTimer();     // Обработчик прерывания таймера 0 по совпадению A 	1 раз в 1 милисекунду
void collect_Data_for_Send(); // Собираем нужные данные и пишем в структуру на отправку
void executeDataReceive();    // Отработка пришедших команд. Изменение скорости, траектории и прочее
template <typename T>
uint32_t measureCheksum(const T &structura_); // Функция возвращает контрольную сумму структуры без последних 4 байтов
// float tfLocalToGlobal360(float _angle, uint8_t _motor); // Функция преобразования из локальной системы координат в глобальную360
// float tfGlobal360ToLocal(float _angle, uint8_t _motor); // Функция преобразования из глобальной 360 в локальную систему моторов
float filtr_My(float old_, float new_, float ves_new_); // Функция фильтрующая(сглаживающая) значения берет старое с меньшим весом и новое с большим
void Led_Blink(int led_, unsigned long time_);          // Функция мигания светодиодом в основном цикле

//************************ РЕАЛИЗАЦИЯ ФУНКЦИЙ *******************************************

// Функция возвращает контрольную сумму структуры без последних 4 байтов
template <typename T>
uint32_t measureCheksum(const T &structura_)
{
  uint32_t ret = 0;
  unsigned char *adr_structura = (unsigned char *)(&structura_); // Запоминаем адрес начала структуры. Используем для побайтной передачи
  for (int i = 0; i < sizeof(structura_) - 4; i++)
  {
    ret += adr_structura[i]; // Побайтно складываем все байты структуры кроме последних 4 в которых переменная в которую запишем результат
  }
  return ret;
}

// Функция фильтрующая(сглаживающая) значения берет старое с меньшим весом и новое с большим
float filtr_My(float old_, float new_, float ves_new_)
{
  return (old_ * (1.0 - ves_new_)) + (new_ * ves_new_);
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
    led_time = millis();
  }
}

// Отработка пришедших команд. Изменение скорости, траектории и прочее
void executeDataReceive()
{
  static int command_pred = 0; // Переменная для запоминания предыдущей команды
  // Команда 0
  if (Data2Modul_receive.controlMotor.command == 0) // Если идет команда  0
  {
    // for (int i = 0; i < 4; i++)
    // {
    //   printf("m%i= %i ", i, digitalRead(motor[i].micric_pin));
    // }
    // printf("\n");
  }
  // printf("com = %i \n", Data2Modul_receive.command);

  // Команда УПРАВЛЕНИЯ УГЛАМИ
  if (Data2Modul_receive.controlMotor.command == 1) // Если пришла команда 2 Управления
  {
    for (int i = 0; i < 4; i++)
    {
      // float angle = tfGlobal360ToLocal(Data2Modul_receive.angle[i], i);
      // printf("motor= %i loc_angle= %f \n", i, angle);
      //  setMotorAngle(i, angle);
      setMotorAngle(i, Data2Modul_receive.controlMotor.angle[i]);
    }
  }

  // Команда КОЛИБРОВКИ И УСТАНОВКИ В 0
  if (Data2Modul_receive.controlMotor.command == 9 && Data2Modul_receive.controlMotor.command != command_pred) // Если пришла команда 9 Колибровки и предыдущая была другая
  {
    setZeroMotor(); // Установка в ноль
  }

  command_pred = Data2Modul_receive.controlMotor.command; // Запоминаяем команду
  //     // printf(" Data2Modul.radius= %f ", Data2Modul_receive.radius);
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
  Serial.println(String(millis()) + " initTimer_0 ...");
  timer0 = timerBegin(0, 80, true);             // Номер таймера, делитель(прескаллер), Считать вверх, прибавлять (true)  Частота базового сигнала 80  Мега герц, значит будет 1 микросекунда
  timerAttachInterrupt(timer0, &onTimer, true); // Какой таймер используем, какую функцию вызываем,  тип прерывания  edge или level interrupts
  timerAlarmWrite(timer0, 1000, true);          // Какой таймер, до скольки считаем , сбрасываем ли счетчик при срабатывании 1000 микросекунд эти 1 милисекунда
  timerAlarmEnable(timer0);                     // Запускаем таймер
}

// Собираем нужные данные и пишем в структуру на отправку
void collect_Data_for_Send()
{
  Modul2Data_send.id++;
  Modul2Data_send.pinMotorEn = digitalRead(PIN_Motor_En); // Считываем состояние пина драйверов

  for (int i = 0; i < 4; i++)
  {
    Modul2Data_send.motor[i].status = motor[i].status; // Считываем состояние пина драйверов
    // Modul2Data_send.motor[i].position = tfLocalToGlobal360(getAngle(motor[i].position), i);       // Записываем текущую позицию преобразуя из импульсов в градусы, надо еще в глобальную систему преобразовывать
    // Modul2Data_send.motor[i].destination = tfLocalToGlobal360(getAngle(motor[i].destination), i); // Считываем цель по позиции, надо еще в глобальную систему преобразовывать
    Modul2Data_send.motor[i].position = getAngle(motor[i].position);       // Записываем текущую позицию преобразуя из импульсов в градусы, надо еще в глобальную систему преобразовывать
    Modul2Data_send.motor[i].destination = getAngle(motor[i].destination); // Считываем цель по позиции, надо еще в глобальную систему преобразовывать

    Modul2Data_send.laser[i].status = laser[i].status;     // Считываем статаус дальномера
    Modul2Data_send.laser[i].distance = laser[i].distance; // Считываем измерение растояния
    Modul2Data_send.laser[i].angle = laser[i].angle;       // Считываем угол в котором произмели измерение

    Modul2Data_send.micric[i] = digitalRead(motor[i].micric_pin); //
  }
  Modul2Data_send.spi.all = spi.all;
  Modul2Data_send.spi.bed = spi.bed;

  Modul2Data_send.cheksum = measureCheksum(Modul2Data_send); // Вычисляем контрольную сумму структуры и пишем ее значение в последний элемент
}

// // Функция преобразования из локальной системы координат в глобальную360
// float tfLocalToGlobal360(float _angle, uint8_t _motor)
// {
//   float angle = _angle + motor[_motor].globalTransform; // К локальной системе прибавляем константу как установлен мотор
//   if (angle > 360)
//   {
//     angle = angle - 360.0;
//   }
//   return angle;
// }

// // Функция преобразования из глобальной 360 в локальную систему моторов
// float tfGlobal360ToLocal(float _angle, uint8_t _motor)
// {
//   float angle = _angle - motor[_motor].globalTransform; // К локальной системе прибавляем константу как установлен мотор
//   if (angle < 0)
//   {
//     angle = angle + 360;
//   }
//   return angle;
// }

void printBody()
{

  printf(" id= %i \n", Modul2Data_send.id);
  // printf(" capacity_real= %f \n", Modul2Data_send.ina.capacity_real);
  printf(" Send cheksum= %i  \n --- \n", Modul2Data_send.cheksum);
}

#ifdef LASER
// Обработка датчиков так что-бы не задерживать основной цикл и делать все короткими операциями
void laserLoop()
{
  static bool flagBroadcastRequest = true;              // Флаг можно ли делать запрос на измерение
  static unsigned long timeBroadcastRequest = millis(); // Время когда сделали запрос/ Достаточно точности в милисекундах
  static bool flagGetStatus = true;                     // Флаг можно ли делать запрос на измерение
  static unsigned long timeGetStatus = millis();        // Время когда сделали запрос/ Достаточно точности в милисекундах
  static int i = 0;

  if (flagBroadcastRequest) // Если нет измерения то меряем
  {
    flagBroadcastRequest = false;            // смена флага
    sk60plus[0].getBroadcastSingleMeasure(); // Щироковещательный запрос на измерние
    timeBroadcastRequest = millis();         // Запоминаем когда сделали запрос
    i = 0;                                   // Номер лазера с которым работаем
    printf("--- \n");
  }
  if ((flagBroadcastRequest == false) && (millis() >= timeBroadcastRequest + 150)) // Через 150 милисекунд попадем сюда после начала измерения
  {
    if (flagGetStatus) // Если можно запрашивать статус
    {
      flagGetStatus = false;    // Смена статуса
      sk60plus[i].getStatus();  // Запрашиваем статус датчика. Ответ будет через 2 миллисекунды
      timeGetStatus = millis(); // Запоминаем когда сделали запрос
    }

    if ((flagGetStatus == false) && (millis() >= timeGetStatus + 5)) // Если сделали запрос и прошло 2 милисекунды
    {
      if (sk60plus[i].readStatus()) // Читаем статус Если ответ по статусу хороший то
      {
        sk60plus[i].getMeasureResult(); // Запрашиваем результат
        delay(5);
        sk60plus[i].readMeasureResult(); //  Считываем результат измерений
        flagGetStatus = true;            // Разрешаем новый запрос на считывание статуса
        i++;                             // Переходим к следующему датчику
        if (i == 4)                      // Обработали последний датчик
        {
          flagBroadcastRequest = true; // Разрешаем новый широковещвтельный запрос на измерение и все начинается по кругу
        }
      }
      else
        flagGetStatus = true; // Разрешаем сделать еще новый запрос
    }
  }
}
#endif

#endif