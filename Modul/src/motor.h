#ifndef MOTOR_H
#define MOTOR_H

//---------------------------------------------------------------------------------------
#define SPEED 1   // Скорость на всех моторах одинаковая максимально возможная, что-бы перемещаться как можно быстрее оборотов за секунду rps
#define MICROSTEP 8 // Микрошаг на драйверах
#define REDUKTOR 4 // Параметры Редуктора
//---------------------------------------------------------------------------------------
#define PIN_Motor_En 32 //
//---------------------------------------------------------------------------------------
#define PIN_M0_Step 33 //
#define PIN_M0_Dir 25  //
//---------------------------------------------------------------------------------------
#define PIN_M1_Step 26 //
#define PIN_M1_Dir 27  //
//---------------------------------------------------------------------------------------
#define PIN_M2_Step 14 //
#define PIN_M2_Dir 12  //
//---------------------------------------------------------------------------------------
#define PIN_M3_Step 15 //
#define PIN_M3_Dir 13  //
//---------------------------------------------------------------------------------------
// #define TRANSFORM_M0 225  // // Углы для трансформации локальной системы в глобальную, зависят от места установки мотора. УКАЗЫВАЮ КУДА СМОТРИТ 0 локальной системы в системе координат 360 градусов плюс по часовой
// #define TRANSFORM_M1 315  //
// #define TRANSFORM_M2 45  //
// #define TRANSFORM_M3 135  //
//---------------------------------------------------------------------------------------
#define PIN_MICRIC_M0 39  //
#define PIN_MICRIC_M1 36  //
#define PIN_MICRIC_M2 34  //
#define PIN_MICRIC_M3 35  //

//************************ ОБЬЯВЛЕНИЕ ФУНКЦИЙ *******************************************
void IRAM_ATTR onTimer1();
int iiRound(double number);                    // ИИ функция округления до целого
void initMotor();                              // Функция инциализации моторов
int32_t getPulse(float _angle);                // Пересчет градусов в импульсы
float getAngle(int32_t _pulse);                // Пересчет импульсов в градусы
void setMotorAngle(int32_t num, float _angle); // Установка мотора в нужное положение в градусах
void setSpeedMotor(float _speed);              // Функция устанавлявающая скорость вращения на ВСЕХ моторах, задается в оборотах за секунду rps
void disableMotor();                           // Отключение моторов в простое

//***************************************************************************************

const float sector = 360.0 / 200 / REDUKTOR / MICROSTEP; // Столько градусов приходится на 1 импульс

int32_t countPulse = 0; // Счетчик импульсов
//bool flagPrintInterrupt = false; // Флаг в режиме колибровки включать печать срабатывания прерывния

//bool flag_command_stop_motor = false; // Флаг что команда остановиться была и нужно отследить время чтобы их отключить
//uint64_t time_command_stop_motor = 0; // Время в которое дали команду остановиться моторам


int microStep;   // Число микрошагов
int timeingStep; // Число тактов для таймера через которое нужно дать новый имульс мотору

int statusTestMotor = 0; // Статус теста мотора для отладки

// ИИ функция округления до целого
int iiRound(double number)
{
    return static_cast<int>(number > 0 ? number + 0.5 : number - 0.5);
}

// Функция инциализации моторов
void initMotor()
{
    //**************************************************************   Мотор на колеса
    microStep = MICROSTEP; // Так распаяно на плате для драйверов 2208 и 2209
    printf("Init StepMotor. Set microstep = %i \n", microStep);

    pinMode(PIN_Motor_En, OUTPUT); // Установка пина разрешающего работу драйвероы
    digitalWrite(PIN_Motor_En, 1); // 0- Разрешена работа 1- запрещена работа драйвера

    pinMode(PIN_M0_Step, OUTPUT); // Устанавливаем пины для M1 мотора
    digitalWrite(PIN_M0_Step, 0); // Подтяжка чтобы не в воздухе
    motor[0].step_pin = PIN_M0_Step;
    pinMode(PIN_M0_Dir, OUTPUT);
    digitalWrite(PIN_M0_Dir, 0); //  Подтяжка чтобы не в воздухе сделал резистором на плате что-бы при перезагрузке не дергалось
    motor[0].dir_pin = PIN_M0_Dir;

    pinMode(PIN_M1_Step, OUTPUT); // Устанавливаем пины для M4  мотора
    digitalWrite(PIN_M1_Step, 0); // Подтяжка чтобы не в воздухе
    motor[1].step_pin = PIN_M1_Step;
    pinMode(PIN_M1_Dir, OUTPUT);
    digitalWrite(PIN_M1_Dir, 0); //  Подтяжка чтобы не в воздухе сделал резистором на плате что-бы при перезагрузке не дергалось
    motor[1].dir_pin = PIN_M1_Dir;

    pinMode(PIN_M2_Step, OUTPUT); // Устанавливаем пины для M2 мотора
    digitalWrite(PIN_M2_Step, 0); // Подтяжка чтобы не в воздухе
    motor[2].step_pin = PIN_M2_Step;
    pinMode(PIN_M2_Dir, OUTPUT);
    digitalWrite(PIN_M2_Dir, 0); //  Подтяжка чтобы не в воздухе сделал резистором на плате что-бы при перезагрузке не дергалось
    motor[2].dir_pin = PIN_M2_Dir;

    pinMode(PIN_M3_Step, OUTPUT); // Устанавливаем пины для M3 мотора 
    digitalWrite(PIN_M3_Step, 0); // Подтяжка чтобы не в воздухе
    motor[3].step_pin = PIN_M3_Step;
    pinMode(PIN_M3_Dir, OUTPUT);
    digitalWrite(PIN_M3_Dir, 0); //  Подтяжка чтобы не в воздухе сделал резистором на плате что-бы при перезагрузке не дергалось
    motor[3].dir_pin = PIN_M3_Dir;

    pinMode(PIN_MICRIC_M0, INPUT_PULLUP); // 
    pinMode(PIN_MICRIC_M1, INPUT_PULLUP); // 
    pinMode(PIN_MICRIC_M2, INPUT_PULLUP); // 
    pinMode(PIN_MICRIC_M3, INPUT_PULLUP); // 

    motor[0].micric_pin = PIN_MICRIC_M0;
    motor[1].micric_pin = PIN_MICRIC_M1;
    motor[2].micric_pin = PIN_MICRIC_M2;
    motor[3].micric_pin = PIN_MICRIC_M3;

    motor[0].status = false; // Флаг ставим что мотор не работает, просто запрещаем делать импульсы
    motor[1].status = false; // Флаг ставим что мотор не работает, просто запрещаем делать импульсы
    motor[2].status = false; // Флаг ставим что мотор не работает, просто запрещаем делать импульсы
    motor[3].status = false; // Флаг ставим что мотор не работает, просто запрещаем делать импульсы
}
// Прерывания на концевиках. ИСПРАВЛЯТЬ ВРУЧНУЮ под реальные пины на плате!!!!
void IRAM_ATTR ISR34()
{
    motor[2].status = false;
}
void IRAM_ATTR ISR35()
{
    motor[3].status = false;
}
void IRAM_ATTR ISR36()
{
    motor[1].status = false;
}
void IRAM_ATTR ISR39()
{
    motor[0].status = false;
}

// Функция исполняемая по прерыванию по таймеру 1 на все МОТОРЫ
void IRAM_ATTR onTimer1() // Обработчик прерывания таймера 0 по совпадению A
{
    digitalWrite(2, 1);
    // countPulse++;

    for (int i = 0; i < 4; i++)
    {
        if (motor[i].status)
        {
            if (motor[i].position == motor[i].destination && statusTestMotor == false) // Статус statusTestMotor только для отладки чтобы включить моторы на постоянное вращение
            {
                motor[i].status = 0;
            }
            else
            {
                digitalWrite(motor[i].step_pin, 1);                              // Если флаг вращения моторов включен тогда делаем импульс
                (motor[i].dir == 1) ? motor[i].position++ : motor[i].position--; // Если считаем шаги
            }
        }
    }

    delayMicroseconds(1);

    for (int i = 0; i < 4; i++)
    {
        digitalWrite(motor[i].step_pin, 0); // Отключаем импульс, делаем всегда на всех пинах без проверки включали ли его
    }

    digitalWrite(2, 0);
}

// Пересчет градусов в импульсы
int32_t getPulse(float _angle)
{
    return iiRound(_angle / sector);
}

// Пересчет импульсов в градусы
float getAngle(int32_t _pulse)
{
    return _pulse * sector;
}

// Установка мотора в нужное положение в градусах локальной системы
void setMotorAngle(int32_t num, float _angle)
{
    if (_angle < 0)
        _angle = 0;                            // Защита от отрицательного градуса угла
    if (_angle > 180)
        _angle = 180;                          // Защита от отклонения больше предела 
    motor[num].destination = getPulse(_angle); // Получаем в какую позицию должен встать мотор наиболее близкую к требуемому градусу
    if (motor[num].position == motor[num].destination) // Если текущая позиция и так тавна цели то ничего не делаем и выходим из функции
        return;
    digitalWrite(PIN_Motor_En, 0);             // Включаем драйвера
    printf("position= %i ", motor[num].position);
    printf("destination= %i \n", motor[num].destination);
    if (motor[num].position < motor[num].destination) // Если цель бпльше то вращение по часовой 1
    {
        digitalWrite(motor[num].dir_pin, 1);
        motor[num].dir = 1;
    }
    if (motor[num].position > motor[num].destination) // Если цель меньше то вращение против часовой 0
    {
        digitalWrite(motor[num].dir_pin, 0);
        motor[num].dir = 0;
    }
    //printf("dir_pin motor %i = %i \n", num, motor[num].dir_pin);
    motor[num].status = 1; // Включаем сам мотор
}

// Функция устанавлявающая скорость вращения на ВСЕХ моторах, задается в оборотах за секунду rps
void setSpeedMotor(float _speed)
{
    printf("setSpeedMotor= %f rps \n", _speed);

    // Скорость в оборотах их умножаем на градусы и делим на градус на 1 шаг получаем нужное число полных шагов для такой скорости за секунду
    // Умножаем на передаточное число редуктора и микрошаг
    float step_za_sec = abs(_speed) * (360 / 1.8) * REDUKTOR * MICROSTEP;
    printf("step_za_sec= %f \n", (float)step_za_sec);

    // Микросекунды в секунде делим на число шагов которые надо успеть сделать за секунду и делим на микрошаги делим на микросекунды за 1 шаг с учетом предделителя таймера
    timeingStep = (float)1000000 / step_za_sec; // Таймер по 1 микросекунде
    printf("timeingStep= %i microsecond \n", timeingStep);
}

// Запуск моторов на тест
void testMotorRun()
{
    digitalWrite(PIN_Motor_En, 0); // Включаем драйвера
    statusTestMotor = 1; // Статус теста мотора Включаем что тест

    motor[0].status = 1;
    motor[1].status = 1;
    motor[2].status = 1;
    motor[3].status = 1;
    Serial.println("testMotorRun...");
    while (1);
}
// Запуск моторов на тест
void testMotorStop()
{
    motor[0].status = 0;
    motor[1].status = 0;
    motor[2].status = 0;
    motor[3].status = 0;
    // digitalWrite(PIN_Motor_En, 1); // Выключаем драйвера
    Serial.println("testMotorStop...");
}

// Инициализация таймера 1. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void initTimer_1()
{
    Serial.println(String(millis()) + " initTimer_1 ...");
    timer1 = timerBegin(1, 80, true);              // Номер таймера, делитель(прескаллер), Считать вверх, прибавлять (true)  Частота базового сигнала 80  Мега герц, значит будет 1 микросекунда
    timerAttachInterrupt(timer1, &onTimer1, true); // Какой таймер используем, какую функцию вызываем,  тип прерывания  edge или level interrupts
    timerAlarmWrite(timer1, timeingStep, true);    // Какой таймер, до скольки считаем , сбрасываем ли счетчик при срабатывании. Значение посчитали когда скороть вращения расчитывали
    timerAlarmEnable(timer1);                      // Запускаем таймер
}


// Инициализация прерывния на концевиках
void initInterrupt()
{
    Serial.println(String(millis()) + " initInterrupt ...");
    pinMode(34, INPUT);
    attachInterrupt(34, ISR34, FALLING);
    pinMode(35, INPUT);
    attachInterrupt(35, ISR35, FALLING);
    pinMode(36, INPUT);
    attachInterrupt(36, ISR36, FALLING);
    pinMode(39, INPUT);
    attachInterrupt(39, ISR39, FALLING);
}

// Отключение моторов в простое
void disableMotor()
{
    if (motor[0].status == 0 && // Если все статусы выключены, значит моторы не двигаются и можно выключать
        motor[1].status == 0 &&
        motor[2].status == 0 &&
        motor[3].status == 0)
    {
        digitalWrite(PIN_Motor_En, 1); // Выключаем драйвера
    }
}

// Функция установки в ноль всех моторов
void setZeroMotor()
{
    Serial.println(String(micros()) + " setZeroMotor ...");
    for (int i = 0; i < 4; i++) // Сначала отводим немного на случай если уже в нуле
    {
        setMotorAngle(i, 15);
    }
    delay(500);
    for (int i = 0; i < 4; i++)
    {
        motor[i].position = 100 * REDUKTOR * MICROSTEP; // Устанавливаем всем максимульную позицию 200 шагов пополам на редуктор и на микрошаг
        setMotorAngle(i, 0);
    }
    delay(1500);                // Время что-бы успел доехать до концевика
    for (int i = 0; i < 4; i++) // По итогу обнуляем позицию и назначение
    {
        motor[i].position = 0;    // Устанавливаем начальную позицию
        motor[i].destination = 0; // Устанавливаем начальную позицию
    }
}

#endif