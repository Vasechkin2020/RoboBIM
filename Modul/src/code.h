#ifndef CODE_H
#define CODE_H

//************************ ОБЬЯВЛЕНИЕ ФУНКЦИЙ *******************************************

void IRAM_ATTR onTimer();     // Обработчик прерывания таймера 0 по совпадению A 	1 раз в 1 милисекунду
void collect_Data_for_Send(); // Собираем нужные данные и пишем в структуру на отправку
void executeDataReceive();    // Отработка пришедших команд. Изменение скорости, траектории и прочее
template <typename T>
uint32_t measureCheksum(const T &structura_);           // Функция возвращает контрольную сумму структуры без последних 4 байтов
float tfLocalToGlobal360(float _angle, uint8_t _motor); // Функция преобразования из локальной системы координат в глобальную360
float tfGlobal360ToLocal(float _angle, uint8_t _motor); // Функция преобразования из глобальной 360 в локальную систему моторов
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
  if (Data2Modul_receive.command == 0) // Если идет команда  0
  {
    // for (int i = 0; i < 4; i++)
    // {
    //   printf("m%i= %i ", i, digitalRead(motor[i].micric_pin));
    // }
    // printf("\n");
  }

  // Команда КОЛИБРОВКИ И УСТАНОВКИ В 0
  if (Data2Modul_receive.command == 1 && Data2Modul_receive.command != command_pred) // Если пришла команда 2 Колибровки и предыдущая была другая
  {
    setZeroMotor(); // Установка в ноль
  }

  // Команда УПРАВЛЕНИЯ УГЛАМИ
  if (Data2Modul_receive.command == 2) // Если пришла команда 2 Управления
  {
    for (int i = 0; i < 4; i++)
    {
      float angle = tfGlobal360ToLocal(Data2Modul_receive.angle[i], i);
      printf("motor= %i loc_angle= %f \n", i, angle);
      setMotorAngle(i, angle);
    }
  }

  command_pred = Data2Modul_receive.command; // Запоминаяем команду
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
    Modul2Data_send.motor[i].status = motor[i].status;                                            // Считываем состояние пина драйверов
    Modul2Data_send.motor[i].position = tfLocalToGlobal360(getAngle(motor[i].position), i);       // Записываем текущую позицию преобразуя из импульсов в градусы, надо еще в глобальную систему преобразовывать
    Modul2Data_send.motor[i].destination = tfLocalToGlobal360(getAngle(motor[i].destination), i); // Считываем цель по позиции, надо еще в глобальную систему преобразовывать

    Modul2Data_send.lidar[i].status = lidar[i].status;     // Считываем статаус дальномера
    Modul2Data_send.lidar[i].distance = lidar[i].distance; // Считываем измерение растояния
    Modul2Data_send.lidar[i].angle = lidar[i].angle;       // Считываем угол в котором произмели измерение

    Modul2Data_send.micric[i] = digitalRead(motor[i].micric_pin); //
  }
  Modul2Data_send.spi.all = spi.all;
  Modul2Data_send.spi.bed = spi.bed;

  Modul2Data_send.cheksum = measureCheksum(Modul2Data_send); // Вычисляем контрольную сумму структуры и пишем ее значение в последний элемент
}

// Функция преобразования из локальной системы координат в глобальную360
float tfLocalToGlobal360(float _angle, uint8_t _motor)
{
  float angle = _angle + motor[_motor].globalTransform; // К локальной системе прибавляем константу как установлен мотор
  if (angle > 360)
  {
    angle = angle - 360.0;
  }
  return angle;
}

// Функция преобразования из глобальной 360 в локальную систему моторов
float tfGlobal360ToLocal(float _angle, uint8_t _motor)
{
  float angle = _angle - motor[_motor].globalTransform; // К локальной системе прибавляем константу как установлен мотор
  if (angle < 0)
  {
    angle = angle + 360;
  }
  return angle;
}

void printBody()
{

  printf(" id= %i \n", Modul2Data_send.id);
  // printf(" capacity_real= %f \n", Modul2Data_send.ina.capacity_real);
  printf(" Send cheksum= %i  \n --- \n", Modul2Data_send.cheksum);
}

#endif