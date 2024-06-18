#ifndef CODE_H
#define CODE_H

void initPIN()
{
  pinMode(PIN_LED_RED, OUTPUT); // Не работает пин. При инициализации SPI_slave функция его занимает и дает всегда половину напряжения и он не реагирует на команды.
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_ANALIZ, OUTPUT);
  pinMode(36, INPUT); // Нет внутренних подтяжек на 4 пинах, которые только на вход!!!!!!!!!!!!!!!!!!

  digitalWrite(PIN_LED_RED, 1);  // Запустился SetUp
  digitalWrite(PIN_LED_BLUE, 1); // Запустился SetUp
  digitalWrite(PIN_ANALIZ, 1);   // Запустился SetUp
}

void offPIN()
{
  digitalWrite(PIN_LED_RED, 0);  // Запустился SetUp
  digitalWrite(PIN_LED_BLUE, 0); // Запустился SetUp
  digitalWrite(PIN_ANALIZ, 0);   // Запустился SetUp
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
void executeCommand()
{
  controlPrint = Data2Print_receive.controlPrint; // Сохраняем в переменную пришедший режим печати
  uint64_t interval = 0;
  if (controlPrint.speed != 0 && controlPrint.intensity != 0)
  {
    interval = abs((float)1000000 / (controlPrint.speed * 1000 * controlPrint.intensity)); // Метры переводим в милиметры, далее умножаем на сколько раз мы хотим прыснуть картриджем на 1 мм, получаем сколько раз нужно прыснуть в секунду,превращаем в микросекунды, так как нас таймер на микросекунду
    if (interval > 1000000)                                                                // Если очень большой на очень маленькой скорости то устанавливам 1 раз в секунду или 1 000 000 микросекунд
      interval = 1000000;
  }
  else
  {
    timerAlarmWrite(timer1, 1000, true); // Устанавливаем период прерывания по таймеру
  }
}

// Функция исполняемая по прерыванию по таймеру 0
void IRAM_ATTR onTimer() // Обработчик прерывания таймера 0 по совпадению A 	1 раз в 1 милисекунду
{
  count_timer_10millisec++;
  count_timer_50millisec++;
  count_timer_1sec++;

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
}
// Функция исполняемая по прерыванию по таймеру 1
void IRAM_ATTR onTimer1() // Обработчик прерывания таймера 0 по совпадению A
{
  if (controlPrint.status == 1) // Если Status равен 1 печатаем, если нулю то нет
  {
    sendSPI(code48Arr[controlPrint.mode], 48); // Какой массив отправлять и размер массива                // Check if transmission was successful
  }

  // printLine(line15_1,36);
  // printLine(line15_0,36);
  // printLine(line15_4,500);
  // printLine(line15_1,36);
  // printLine(line15_0,36);
  // printLine(line15_2,36);
  // printLine(line15_3,36);
  // printLine(line15_2,36);
  // printLine(line15_0,36);
  // printLine(line15_1,36);

  // kartr.code24toCode48(code24free, code48free);
  // sendSPI(code48free, 48); // Какой массив отправлять и размер массива                // Check if transmission was successful

  // if (flag_motor_L_EN) // Если флаг вращения моторов включен тогда делаем импульс
  // {
  //     delayMicroseconds(1);
  //     timerAlarmWrite(timer1, Timeing_Step_L, true); // Устанавливаем период прерывания по таймеру
  //     set_odom_impuls(odom_dir_L, odom_impuls_L);    // Считаем ипульсы для одометрии типа как энкодер
  //     digitalWrite(PIN_L_Step, 0);
  // }
}

// Инициализация таймера 0. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void initTimer_0()
{
  printf(" Starrt initTimer_0 ... \n");
  timer0 = timerBegin(0, 80, true);             // Номер таймера, делитель(прескаллер), Считать вверх, прибавлять (true)  Частота базового сигнала 80  Мега герц, значит будет 1 микросекунда
  timerAttachInterrupt(timer0, &onTimer, true); // Какой таймер используем, какую функцию вызываем,  тип прерывания  edge или level interrupts
  timerAlarmWrite(timer0, 1000, true);          // Какой таймер, до скольки считаем , сбрасываем ли счетчик при срабатывании 1000 микросекунд это 1 милисекунда
  timerAlarmEnable(timer0);                     // Запускаем таймер
}
// Инициализация таймера 1. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void initTimer_1()
{
  printf(" Start initTimer_1 ... \n");
  timer1 = timerBegin(1, 80, true);              // Номер таймера, делитель(прескаллер), Считать вверх, прибавлять (true)  Частота базового сигнала 80  Мега герц, значит будет 1 микросекунда
  timerAttachInterrupt(timer1, &onTimer1, true); // Какой таймер используем, какую функцию вызываем,  тип прерывания  edge или level interrupts
  timerAlarmWrite(timer1, 1000, true);           // Какой таймер, до скольки считаем , сбрасываем ли счетчик при срабатывании. 1000 микросекунд эти 1 милисекунда
  timerAlarmEnable(timer1);                      // Запускаем таймер
  printf("%lu initTimer_1 Ok. \n", millis());
}

// Собираем нужные данные и пишем в структуру на отправку
void collect_Data()
{
  Print2Data_send.id++;

  Print2Data_send.spi.all = obmen_all;
  Print2Data_send.spi.bed = obmen_bed_crc; // + obmen_bed_time;

  Print2Data_send.cheksum = measureCheksum(Print2Data_send); // Вычисляем контрольную сумму структуры и пишем ее значение в последний элемент

  // Serial.print(" Iot2Data_send.cheksum = ");
  // Serial.println(Iot2Data_send.cheksum);
}
void printBody()
{

  printf(" id= %i \n", Print2Data_send.id);
  printf(" Send cheksum= %i  \n --- \n", Print2Data_send.cheksum);
}
// Печать какой строки и сколько раз
void printLine(uint8_t *line_, uint16_t count_)
{

  kartr.line15toCode24(line_, code24);
  kartr.code24toCode48(code24, code48);
  for (int i = 0; i < count_; i++)
  {
    sendSPI(code48, 48); // Какой массив отправлять и размер массива                // Check if transmission was successful
    delayMicroseconds(250);
  }

  // memset(&t, 0, sizeof(t));      // Zero out the transaction
  // t.length = sizeof(code48) * 8; // Length is in bits (48 bytes * 8 bits/byte)
  // t.tx_buffer = code48;          // Data

  // // Начало передачи данных по SPI
  // ret = spi_device_transmit(handle, &t); // Transmit!
  // assert(ret == ESP_OK);                 // Check if transmission was successful

  // sendSPI(code48,sizeof(code48)); // Какой массив отправлять и размер массива
  // delayMicroseconds(250);
}

// Заполнение массива строками варианты печати
void initLineArray()
{
  kartr.line15toCode24(line15_0, code24);
  kartr.code24toCode48(code24, code48);
  kartr.code48toCode48Arr(code48, code48Arr[0]);

  kartr.line15toCode24(line15_1, code24);
  kartr.code24toCode48(code24, code48);
  kartr.code48toCode48Arr(code48, code48Arr[1]);

  kartr.line15toCode24(line15_2, code24);
  kartr.code24toCode48(code24, code48);
  kartr.code48toCode48Arr(code48, code48Arr[2]);

  kartr.line15toCode24(line15_3, code24);
  kartr.code24toCode48(code24, code48);
  kartr.code48toCode48Arr(code48, code48Arr[3]);

  kartr.line15toCode24(line15_4, code24);
  kartr.code24toCode48(code24, code48);
  kartr.code48toCode48Arr(code48, code48Arr[4]);

  kartr.line15toCode24(line15_5, code24);
  kartr.code24toCode48(code24, code48);
  kartr.code48toCode48Arr(code48, code48Arr[5]);

  // kartr.printLine(line10, 10);
  // Serial.println(String(micros()) + "AAAA " + line10[0]);
  // printf("ppppp");
  // kartr.line10to150(line10, line150);
  // kartr.printLine(line150, 150);

  // kartr.line15toCode24(line15_1, code24);
  // kartr.code24toCode48(code24, code48);

  // kartr.line15toCode24(line15_0, code24null);
  // kartr.code24toCode48(code24null, code48null);

  // Serial.println(" === ");
  // for (int i = 0; i < 48; i++)
  // {
  //   Serial.print(" i= ");
  //   Serial.print(i);
  //   Serial.print(" = ");
  //   Serial.println(code48Arr[1][i], HEX);
  // }
  // Serial.println(" === ");

  // u_int8_t *code48 = (uint8_t *)code24; // Создаем переменную в которую пишем адрес буфера в нужном формате
  //                                       // code48Send = *code48;                // Копируем из этой перемнной данные в мою структуру

  // Serial.println(" =code48= ");
  // for (int i = 0; i < 48; i++)
  // {
  //     Serial.print(" i= ");
  //     Serial.print(i);
  //     Serial.print(" = ");
  //     Serial.println(code48free[i], HEX);
  // }
}

#endif