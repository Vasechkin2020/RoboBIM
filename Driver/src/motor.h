#ifndef MOTOR_H
#define MOTOR_H

//************************ ОБЬЯВЛЕНИЕ ФУНКЦИЙ *******************************************
void set_odom_impuls(int8_t odom_dir_, int16_t &odom_impuls_); // Считаем одометрию Амперсанд перед переменной значит ссылка на нее
void IRAM_ATTR onTimer1();                                     // Обработчик прерывания таймера 0 по совпадению A // Функция исполняемая по прерыванию по таймеру 1 ЛЕВЫЙ МОТОР
void IRAM_ATTR onTimer2();                                     // Обработчик прерывания таймера 0 по совпадению A // Функция исполняемая по прерыванию по таймеру 2 ПРАВЫЙ МОТОР
float chekRPS(float _rps);                                     // Проверка и ограничение минимальной и максимальной частоты вращения. Если меньше минимума то считаем что ноль

void setSpeed_L(float _speed); // Функция устанавлявающая скорость вращения на правом моторе
void setSpeed_R(float _speed); // Функция устанавлявающая скорость вращения на правом моторе

void initTimer_1();       // Инициализация таймера 1. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void initTimer_2();       // Инициализация таймера 1. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void initMotor();         // Инициализация пинов моторов и установка начальных значений
void testMotor();         // Функция тестирования правильности подключения моторов.
void delayMotor();        // Функция задержки выключения моторов после команды стоп на 1 секунду для экономии энергии и чтобы не грелись
void movementTime();      // Функция отслеживает время движения. Движемся только опредленное время и потом если нет новой команды останавливаемся
void calculateOdom_enc(); // Функция обсчета телеметрии колес. перобразуем ипульсы в метры пройденного пути с учетом направления вращения
//***************************************************************************************
//---------------------------------------------------------------------------------------
#define PIN_En 14 //
//---------------------------------------------------------------------------------------
#define PIN_L_Step 26 //
#define PIN_L_Dir 12  //
//---------------------------------------------------------------------------------------
#define PIN_R_Step 32 //
#define PIN_R_Dir 33  //

#define DIAMETR 150 // Влияет на правильность длинны через расчет скорости
#define RADIUS (DIAMETR / 2)
#define PERIMETR (DIAMETR * PI)

#define MAX_RPS 2.0 // максимальная скорость колеса в оборотах в секунду
//---------------------------------------------------------------------------------------

bool flag_motor_L_EN = 0; // Флаг работает или нет Левый мотор
bool flag_motor_R_EN = 0; // Флаг работает или нет Правый мотор

uint64_t time_command_motor = 0; // Время в которое дали команду остановиться моторам

int microStep;      // Число микрошагов
int Timeing_Step_L; // Число тактов для таймера через которое нужно дать новый имульс мотору
int Timeing_Step_R; // Число тактов для таймера через которое нужно дать новый имульс мотору

// Одометрия
float rps_R_gived = 0; // Скорость на моторе желаемаая rpm
float rps_R_odom = 0; // Скорость на моторе на основе одометрии по импульсам rpm

float rps_L_gived = 0; // Скорость на моторе желаемаая rpm
float rps_L_odom = 0; // Скорость на моторе на основе одометрии по импульсам rpm

int16_t odom_impuls_L = 0; // Количество ипульсов переданных колесу с учетом направления вращения
int16_t odom_impuls_R = 0; // Количество ипульсов переданных колесу с учетом направления вращения

int8_t odom_dir_L = 0; // Направление вращения левого колеса в данный момент 1- вперед, -1 - назад, 0 стоим на месте
int8_t odom_dir_R = 0; // Направление вращения правого колеса в данный момент 1- вперед, -1 - назад, 0 стоим на месте

uint64_t timeExecuteCommand = 0; // время в который момент начали движение
bool flagExecuteCommand = 0;     // Флаг что надо отслеживать движение по времени

// Считаем одометрию Амперсанд перед переменной значит ссылка на нее
void set_odom_impuls(int8_t odom_dir_, int16_t &odom_impuls_)
{
    if (odom_dir_ == 1)
    {
        odom_impuls_++;
    }
    if (odom_dir_ == -1)
    {
        odom_impuls_--;
    }
}
// Функция исполняемая по прерыванию по таймеру 1 ЛЕВЫЙ МОТОР
void IRAM_ATTR onTimer1() // Обработчик прерывания таймера 0 по совпадению A
{
    if (flag_motor_L_EN) // Если флаг вращения моторов включен тогда делаем импульс
    {
        digitalWrite(PIN_L_Step, 1);
        delayMicroseconds(1);
        timerAlarmWrite(timer1, Timeing_Step_L, true); // Устанавливаем период прерывания по таймеру
        set_odom_impuls(odom_dir_L, odom_impuls_L);    // Считаем ипульсы для одометрии типа как энкодер
        digitalWrite(PIN_L_Step, 0);
    }
}
// Функция исполняемая по прерыванию по таймеру 2 ПРАВЫЙ МОТОР
void IRAM_ATTR onTimer2() // Обработчик прерывания таймера 0 по совпадению A
{
    if (flag_motor_R_EN) // Если флаг вращения моторов включен тогда делаем импульс
    {
        digitalWrite(PIN_R_Step, 1);
        delayMicroseconds(1);
        timerAlarmWrite(timer2, Timeing_Step_R, true); // Устанавливаем период прерывания по таймеру
        set_odom_impuls(odom_dir_R, odom_impuls_R);    // Считаем ипульсы для одометрии типа как энкодер
        digitalWrite(PIN_R_Step, 0);
    }
}
// Проверка и ограничение минимальной и максимальной частоты вращения. Если меньше минимума то считаем что ноль
float chekRPS(float _rps)
{
    if (_rps > 0 && _rps < 0.01)
        _rps = 0;
    if (_rps < 0 && _rps > -0.01)
        _rps = 0;
    if (_rps > MAX_RPS)
        _rps = MAX_RPS;
    if (_rps < -MAX_RPS)
        _rps = -MAX_RPS;
    return _rps;
}

// Функция устанавлявающая скорость вращения на правом моторе
void setSpeed_L(float _speed)
{
    float step_za_sec = 0;
    float rps = ((_speed * 1000) / PERIMETR); // Скорость приводим в милиметры и делим на периметр колеса получаем сколько нужно делать оборотов  в секунду для такой скорости за секунду
    rps = chekRPS(rps);                       // Проверка минимального и максимального rps
    time_command_motor = millis();            // Запоминаем время когда выполнили любую команду по управлением скоростью мотора. Потом если пройдет время отключим драйвера delayMotor()
    if (rps != 0)                             // Если не задано что ноль, то считаем начальную скорость и смотрим куда едем
    {
        digitalWrite(PIN_En, 0); // Включаем драйвера
        if (rps < 0)             // Задаем направление вращения
        {
            digitalWrite(PIN_L_Dir, 1);
            odom_dir_L = -1; // Тут назначаем куда вращаемся для одометрии для левого и правого колеса может быть надо по разному
        }
        if (rps > 0)
        {
            digitalWrite(PIN_L_Dir, 0);
            odom_dir_L = 1; // Тут назначаем куда вращаемся для одометрии для левого и правого колеса может быть надо по разному
        }
        motorLeft.rpsSet = rps;                                          // Фиксируем фактичские обороты
        rps = abs(rps); // После того как определили направление, дальше считаем по модулю
        step_za_sec = (rps * 360 / 1.8);                                 // Берем обороты их умножаем на градусы и делим на градус на 1 шаг получаем нужное число полных шагов для такой скорости за секунду
        Timeing_Step_L = (float)1000000 / step_za_sec / microStep / 1.0; // Таймер по 1 микросекунде// Микросекунды в секунде делим на число шагов которые надо успеть сделать за секунду и делим на микрошаги делим на микросекунды за 1 шаг с учетом предделителя таймера
        flag_motor_L_EN = true;                                          // Взводим флаг разрешающий делать импульсы в обработчике прерывания
    }
    else
    {
        flag_motor_L_EN = false; // Флаг ставим что мотор не работает это чтобы импульс не подавался
    }
    //printf("%lu setSpeed_L= %f -> rpsL=  %.3f -> step_za_sec L = %.3f  -> Timeing_Step_L = %i \n", millis(), _speed, rps, step_za_sec, Timeing_Step_L);
    printf("%lu speedL %f rps %.3f stepSec %.3f TStep %i \n", millis(), _speed, rps, step_za_sec, Timeing_Step_L);
}
// Функция устанавлявающая скорость вращения на правом моторе
void setSpeed_R(float _speed)
{
    time_command_motor = millis(); // Запоминаем время когда выполнили любую команду по управлением скоростью мотора. Потом если пройдет время отключим драйвера delayMotor()
    float step_za_sec = 0;
    float rps = ((_speed * 1000) / PERIMETR); // Скорость приводим в милиметры и делим на периметр колеса получаем сколько нужно делать оборотов  в секунду для такой скорости за секунду
    rps = chekRPS(rps);                       // Проверка минимального rps
    if (rps != 0)                             // Если не задано что ноль, то считаем начальную скорость и смотрим куда едем
    {
        digitalWrite(PIN_En, 0); // Включаем драйвера
        if (rps < 0)             // Задаем направление вращения
        {
            digitalWrite(PIN_R_Dir, 0);
            odom_dir_R = -1; // Тут назначаем куда вращаемся для одометрии для левого и правого колеса может быть надо по разному
        }
        if (rps > 0)
        {
            digitalWrite(PIN_R_Dir, 1);
            odom_dir_R = 1; // Тут назначаем куда вращаемся для одометрии для левого и правого колеса может быть надо по разному
        }
        motorRight.rpsSet = rps;                                         // Фиксируем фактичские обороты
        rps = abs(rps); // После того как определили направление, дальше считаем по модулю
        step_za_sec = (rps * 360 / 1.8);                                 // Берем обороты их умножаем на градусы и делим на градус на 1 шаг получаем нужное число полных шагов для такой скорости за секунду
        Timeing_Step_R = (float)1000000 / step_za_sec / microStep / 1.0; // Таймер по 1 микросекунде// Микросекунды в секунде делим на число шагов которые надо успеть сделать за секунду и делим на микрошаги делим на микросекунды за 1 шаг с учетом предделителя таймера
        flag_motor_R_EN = true;                                          // Взводим флаг разрешающий делать импульсы в обработчике прерывания
    }
    else
    {
        flag_motor_R_EN = false; // Флаг ставим что мотор не работает это чтобы импульс не подавался
    }
    //printf("%lu setSpeed_R= %f -> rpsR=  %.3f -> step_za_sec R = %.3f  -> Timeing_Step_R = %i \n", millis(), _speed, rps, step_za_sec, Timeing_Step_R);
    printf("%lu speedR %f rps %.3f stepSec %.3f TStep %i \n", millis(), _speed, rps, step_za_sec, Timeing_Step_R);
}

// Инициализация таймера 1. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void initTimer_1()
{
    timer1 = timerBegin(1, 80, true);              // Номер таймера, делитель(прескаллер), Считать вверх, прибавлять (true)  Частота базового сигнала 80  Мега герц, значит будет 1 микросекунда
    timerAttachInterrupt(timer1, &onTimer1, true); // Какой таймер используем, какую функцию вызываем,  тип прерывания  edge или level interrupts
    timerAlarmWrite(timer1, 100000, true);         // Какой таймер, до скольки считаем , сбрасываем ли счетчик при срабатывании. 1000 микросекунд эти 1 милисекунда
    timerAlarmEnable(timer1);                      // Запускаем таймер
    printf("%lu initTimer_1 \n", millis());
}
// Инициализация таймера 1. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void initTimer_2()
{
    timer2 = timerBegin(2, 80, true);              // Номер таймера, делитель(прескаллер), Считать вверх, прибавлять (true)  Частота базового сигнала 80  Мега герц, значит будет 1 микросекунда
    timerAttachInterrupt(timer2, &onTimer2, true); // Какой таймер используем, какую функцию вызываем,  тип прерывания  edge или level interrupts
    timerAlarmWrite(timer2, 100000, true);         // Какой таймер, до скольки считаем , сбрасываем ли счетчик при срабатывании. 1000 микросекунд эти 1 милисекунда
    timerAlarmEnable(timer2);                      // Запускаем таймер
    printf("%lu initTimer_2 \n", millis());
}

// Инициализация пинов моторов и установка начальных значений
void initMotor()
{
    //**************************************************************   Мотор на колеса
    microStep = 8; // Так распаяно на плате для драйверов 2208 и 2209
    pinMode(PIN_En, OUTPUT);
    digitalWrite(PIN_En, 1); // 0- Разрешена работа 1- запрещена работа драйвера

    pinMode(PIN_L_Step, OUTPUT); // Устанавливаем пины для левого мотора
    digitalWrite(PIN_L_Step, 0); // Подтяжка чтобы не в воздухе
    pinMode(PIN_L_Dir, OUTPUT);
    digitalWrite(PIN_L_Dir, 0); //  Подтяжка чтобы не в воздухе сделал резистором на плате что-бы при перезагрузке не дергалось

    pinMode(PIN_R_Step, OUTPUT); // Устанавливаем пины для правого мотора
    digitalWrite(PIN_R_Step, 0); // Подтяжка чтобы не в воздухе
    pinMode(PIN_R_Dir, OUTPUT);
    digitalWrite(PIN_R_Dir, 0); //  Подтяжка чтобы не в воздухе сделал резистором на плате что-бы при перезагрузке не дергалось

    flag_motor_L_EN = false; // Флаг ставим что мотор не работает, просто перестаем делать импульсы
    flag_motor_R_EN = false; // Флаг ставим что мотор не работает, просто перестаем делать импульсы

    printf("%lu Init StepMotor. Set microstep = %i \n", millis(), microStep);

    initTimer_1(); // Таймер на 1 мотор
    initTimer_2(); // Таймер на 2 мотор
}

// Функция задержки выключения моторов после команды стоп на 1 секунду для экономии энергии и чтобы не грелись
void delayMotor()
{
    // Отслеживания времени отключить драйвера на моторах чтобы не грелись и не потребляли энергию
    if (millis() - time_command_motor > 2000) // Если с момента остановки моторов прошло более 2000 милисекунд (2 секунда) и флаг не изменился другой командой
    {
        digitalWrite(PIN_En, 1); // Выключаем драйвера
    }
}

float angle_pov_sum = 0;
// Функция обсчета телеметрии колес. перобразуем ипульсы в метры пройденного пути с учетом направления вращения
void calculateOdom_enc()
{
    static unsigned long time = micros();       // Время предыдущего расчета
    unsigned long time_now = micros();          // Время в которое делаем расчет
    float dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды
    time = time_now;
    //------------------------------------
    float gradusL = odom_impuls_L * (1.8 / microStep); // На сколько градусов проехали c учетом микрошага драйвера 8
    float gradusR = odom_impuls_R * (1.8 / microStep); // На сколько градусов проехали c учетом микрошага драйвера 8

    odom_impuls_L = 0; // Обнуляем значение
    odom_impuls_R = 0; // Обнуляем значение

    motorLeft.rpsEncod = (gradusL / dt) / 360;  // Сколько оборотов сделали
    motorRight.rpsEncod = (gradusR / dt) / 360; // Сколько оборотов сделали

    // printf("odom_impuls_L= %i rps_L= %f \n", odom_impuls_L, rps_L);
    // printf("odom_impuls_R= %i rps_R= %f \n", odom_impuls_R, rps_R);
    // printf("dt= %f odom_impuls_L= %i impuls for sec= %f rps= %f rps2= %f \n", dt, odom_impuls_L, odom_impuls_L /dt, (gradusL /dt) / 360, rps_L);
    // printf("dt= %f odom_impuls_R= %i impuls for sec= %f rps= %f rps2= %f \n", dt, odom_impuls_R, odom_impuls_R /dt, (gradusR /dt) / 360, rps_R);

    float way_L = ((PI * (RADIUS / 1000.0)) / 180.0) * gradusL; // По формуле длинны дуги находим пройденный путь колесом Радиус приводим в метры так как он указан в мм Это и есть скорость за секунду
    float way_R = ((PI * (RADIUS / 1000.0)) / 180.0) * gradusR; // По формуле длинны дуги находим пройденный путь колесом
    float way_C = (way_L + way_R) / 2.0;                        // Путь средней точки Center
    // motorLeft.way += way_L;                                     // суммируем
    // motorRight.way += way_R;                                    // суммируем
    car.way += way_C; // суммируем

    car.speedEncod = way_C / dt; // Скорость с какой фактически едем это длинна дуги за секунду

    // // Радиус фактический высчитываем из раздницы скоростей колес https://present5.com/presentation/111019976_277103894/image-5.jpg
    // float VV = (way_L + way_R); // Путь это и есть скорость за время
    // float V_V = (way_L - way_R);

    // if (V_V == 0 && V_V < 0.001) // Чтобы не делить на ноль когда скорости равны или очень маленькая разница
    // {
    //     V_V = 0.001;
    // }
    // car.radiusEncod = 0.5 * DISTANCE_WHEELS * (VV / V_V); //
    // if (car.radiusEncod > 0)
    // {
    //     car.radiusEncod = car.radiusEncod + DISTANCE_WHEELS / 2;
    // }
    // else
    // {
    //     car.radiusEncod = car.radiusEncod - DISTANCE_WHEELS / 2;
    // }
    // if (abs(car.radiusEncod) > 5) // Если едем прямо то большой радиус и его превращаем в ноль, чтобы ясно было что едем прямо
    // {
    //     car.radiusEncod = 0;
    // }

    //****************************************** РАСЧЕТ ОДОМЕТРИИ **************************************************************************************************************************
    // printf("dt= %f odom_way_L= %f  odom_way_R= %f odom_way_C= %f \n", dt, motorLeft.way, motorRight.way, car.way);

    // float angle_pov;
    // Находим угловую скорость вращения (поворота) робота
    // if (car.radiusSet == 0) // Если двигаемся прямо то
    // {
    //     angle_pov = 0; // Угол поворота равен нулю
    // }
    // else
    // {
    //     angle_pov = way_C / -car.radiusSet; // Делим пройденный оп дуге путь на радиус движения получаем угол поворота в радианах
    //     angle_pov_sum += angle_pov * 180.0 / PI;
    // }
    // Находим угловую скорость поворота в радианах в секунду
    // odom_enc.vel_th = angle_pov / dt; //  Вычисляем радианы в секунду получаем угловую скорость
    // //---------------------------------------
    // // Находим линейные скорости из моего вектора скорости. Я задаю общую скорость движения (длинна вектора), ее надо разложить на проекции по осям x y. Это будут линейные скорости
    // odom_enc.vel_x = car.speedEncod * cos(odom_enc.th); // Проекция моей скорости на ось X получаем линейную скорость по оси
    // odom_enc.vel_y = car.speedEncod * sin(odom_enc.th); // Проекция моей скорости на ось Y получаем линейную скорость по оси

    // printf("car.speedEncod= %.2f  g_radius= %.2f angle_pov= %f  angle_pov_sum= %f vel_x= %.2f  vel_y= %.2f  vel_th= %f ", car.speedEncod, g_radius, angle_pov, angle_pov_sum, g_odom_enc.vel_x, g_odom_enc.vel_y, g_odom_enc.vel_th);

    // Находим смещение по глобальной карте по осям Х и Y c помощью матрицы вращения. Она вычисляет смешения по осям на нужный угол
    // float delta_x = (odom_enc.vel_x * cos(odom_enc.th) - odom_enc.vel_y * sin(odom_enc.th)) * dt;
    // float delta_y = (odom_enc.vel_x * sin(odom_enc.th) + odom_enc.vel_y * cos(odom_enc.th)) * dt;

    // float delta_x = (odom_enc.vel_x) * dt;
    // float delta_y = (odom_enc.vel_y) * dt;
    // float delta_th = (odom_enc.vel_th) * dt;

    // Меняем координаты и угол на основе вычислений
    // odom_enc.x += delta_x;   // Вычисляем координаты
    // odom_enc.y += delta_y;   // Вычисляем координаты
    // odom_enc.th += delta_th; // Прибавляем к текущему углу и получаем новый угол куда смотрит наш робот

    // printf("x= %.2f y= %.2f th= %.3f  time= %u \n", g_odom_enc.x, g_odom_enc.y, g_odom_enc.th, millis());
}

// float checkMinRadius(float radius_) // Проверка радиуса на реалистичность в соответствии с реальными возможностями робота
// {
//     // printf("checkMinRadius radius_ IN= %f", radius_);
//     //  Проверка радиуса на возможность робота. Радиус не может быть меньше расстояния между колесами
//     float min_r = DISTANCE_WHEELS;
//     if (radius_ > 0 && radius_ < min_r) // радиус не может быть меньше минимума за этим нужно следить на верхнем уровне
//     {
//         radius_ = min_r; // Присваиваем минимально возможный
//     }
//     if (radius_ < 0 && radius_ > -min_r) // радиус не может быть меньше минимума за этим нужно следить на верхнем уровне
//     {
//         radius_ = -min_r; // Присваиваем минимально возможный
//     }
//     // printf(" real= %f \n", radius_);
//     return radius_;
// }

// // Проверка на максимальную скорость учитывая радиус который задан и максимальную скорость моторов с какой они могут вращаться. Проверка по внешнему колесу, как сомому быстрому
// float checkMaxSpeedRadius(float radius_, float speed_)
// {
//     // printf("getRealSpeed speed_ IN= %f", speed_);
//     if (radius_ != 0) // Вычисляем если радиус не ноль (движение прямо)
//     {
//         float radius_centre = abs(radius_) - (DISTANCE_WHEELS / 2.0); // Вычисляем радиус по центру машинки, так как просто радиус это внешний радиус по внешнему колесу.
//         float speed_v = (abs(radius_) * speed_) / radius_centre;      // Вычисляем скорость на внешнем колесе учитывая что задана скорость по центру и учитывая радиус
//         if (speed_v > MAX_SPEED)                                      // Понижаем скорость если скорость на внешнем колесе превысила максимум
//             speed_v = MAX_SPEED;
//         if (speed_v < -MAX_SPEED)
//             speed_v = -MAX_SPEED;
//         speed_ = (speed_v * radius_centre) / abs(radius_); // Обратное вычисление скорости по центру
//     }
//     // printf(" real= %f \n", speed_);
//     return speed_;
// }
// Установка скорости моторов в зависимости от радиуса и направления
// void setSpeedRadius(float speed_, float radius_)
// {
//     car.radiusSet = checkMinRadius(radius_); // Проверка на реалистичность Запоминаяем радиус для расчета одометрии теперь у нас такой радиус отнего все и считаем Это центр робота
//     // Радиус делаем главным. Далее считаем скорость по колесам и если скорость превышает возможную то подгоняем ее по максимуму и едем с возможноц скоростью, но по заданному радиусу
//     car.speedSet = checkMaxSpeedRadius(car.radiusSet, speed_); // Проверка на MAXSPEED Запоминаяем скорость для расчета одометрии теперь у нас такая скорость от нее все и считаем

//     float k2 = 0;
//     if (car.radiusSet != 0) // Вычисляем коефициент перобразования иначе он остается равен 0
//     {
//         // k2 = (DISTANCE_WHEELS / 2.0) / abs(g_radius); // Находим долю какую составляем растояние половинки между колесами тела и радиусом поворота.
//         k2 = (DISTANCE_WHEELS / 2) / (abs(car.radiusSet) - (DISTANCE_WHEELS / 2)); // Находим долю какую составляем растояние половинки между колесами тела и радиусом поворота.
//         // printf("k2= %f \n", k2);
//     }
//     float internal_speed = car.speedSet * (1 - k2);
//     float external_speed = car.speedSet * (1 + k2);
//     // printf(" internal_speed= %f  center_speed= %f external_speed= %f \n", internal_speed, speed_, external_speed);

//     // Определяем направление движения
//     if (car.radiusSet == 0) // Едем по прямой
//     {
//         setSpeed_L(car.speedSet);
//         setSpeed_R(car.speedSet);
//     }
//     if (car.radiusSet < 0) //  едем против часовой стрелки, поворачиваем налево
//     {
//         setSpeed_L(internal_speed); // Левое едет медленней
//         setSpeed_R(external_speed); // Правое едет быстрее
//     }
//     if (car.radiusSet > 0) // По часовой стрелке направо
//     {
//         setSpeed_L(external_speed); // Левое едет быстрее
//         setSpeed_R(internal_speed); // Правое едет медленно
//     }
// }
// Функция отслеживает время движения. Движемся только опредленное время и потом если нет новой команды останавливаемся
void movementTime()
{
    // Отслеживание времени с прихода последней команды по времени
    if (flagExecuteCommand)
    {
        if (millis() - timeExecuteCommand > 1000) // Если секунду нет команд то стоп моторы
        {
            flagExecuteCommand = false; // Заканчиваем отслеживать
            setSpeed_L(0);
            setSpeed_R(0);
        }
    }
}

// Функция тестирования правильности подключения моторов.
void testMotor()
{
    setSpeed_L(0.05); // Задаем напрямую скорость вращения
    setSpeed_R(0.05);
    while (1)
        ;
}
// Расчет коеффициента соотношения ускорений для ускорения в повороте Пример расчета для 1,2 ускорения и скорорстей 9 и 1
// void getKoefAccel()
// {
//     float kL = 0;
//     float kR = 0;
//     acceler_L = 1;                                   //
//     acceler_R = 1;                                   //
//     float d_gived_L = abs(rps_L_gived - rps_L_fact); // Находим разницу скоростей между тем что есть и тем что надо
//     float d_gived_R = abs(rps_R_gived - rps_R_fact);
//     if (d_gived_L != 0 || d_gived_R != 0) // Если хоть одна разница скоростей не равна нулю
//     {
//         float k = (d_gived_L + d_gived_R) / 2; // находим среднюю скорость  9+1 =10/2=5
//         kL = d_gived_L / k;                    // Ускорение на левом моторе 9/5=1.8
//         kR = d_gived_R / k;                    // Ускорение на левом моторе 1/5=0.2

//         if (kL >= kR)
//         {
//             acceler_R = kR / kL; //
//         }
//         else
//         {
//             acceler_L = kL / kR; //
//         }
//         // if (acceler_R < 0 || acceler_R > 1)
//         // {
//         //     Serial.print("============= > ");
//         //     Serial.print(acceler_R);
//         // }
//         // if (acceler_L < 0 || acceler_L > 1)
//         // {
//         //     Serial.print("============= > ");
//         //     Serial.print(acceler_L);
//         // }
//     }
//     // Serial.print(" rps_L_fact= ");
//     // Serial.print(rps_L_fact);
//     // Serial.print(" d_gived_L= ");
//     // Serial.print(d_gived_L);

//     // Serial.print(" !  rps_R_fact= ");
//     // Serial.print(rps_R_fact);
//     // Serial.print(" d_gived_R= ");
//     // Serial.print(d_gived_R);

//     // Serial.print(" !  kL= ");
//     // Serial.print(kL);

//     // Serial.print(" kR= ");
//     // Serial.print(kR);

//     // Serial.print(" !  acceler_L= ");
//     // Serial.print(acceler_L);
//     // Serial.print(" acceler_R= ");
//     // Serial.println(acceler_R);
// }
// Функция ускорения на правом моторе. Смотрит с какими оборотами надо и какие уже установлены и прибавляет или убаваляет обороты
// void setAcceleration_R()
// {
//     if (flag_motor_R_EN) // Если мотор работает то и эта функция отрабатывает
//     {
//         float rps_new = rps_R_fact;                       // По умолчанию оставляем какие есть.
//         float accel = (ACCELERATION * acceler_R) / 100.0; // Ускорение

//         if (rps_R_fact < rps_R_gived) // Если меньше чем надо то прибавим оборотов
//         {
//             rps_new = rps_R_fact + (accel);
//         }
//         if (rps_R_fact > rps_R_gived) // Если больше чем надо то убавим оборотов
//         {
//             rps_new = rps_R_fact - (accel);
//             if (rps_new < 0.001) // Если совсем уменьшили меньше минимума
//             {
//                 rps_new = 0;
//             }
//         }
//         setTimeing_Step_R(rps_new);

//         Serial.print(" rps_new R= ");
//         Serial.print(rps_new);
//     }
// }

// Функция ускорения на правом моторе. Смотрит с какими оборотами надо и какие уже установлены и прибавляет или убаваляет обороты
// void setAcceleration_L()
// {
//     if (flag_motor_L_EN) // Если мотор работает то и эта функция отрабатывет
//     {
//         float rps_new = rps_L_fact;                       // По умолчанию оставляем какие есть.
//         float accel = (ACCELERATION * acceler_L) / 100.0; // Ускорение
//         if (rps_L_fact < rps_L_gived)                     // Если меньше чем надо то прибавим оборотов
//         {
//             rps_new = rps_L_fact + (accel);
//         }
//         if (rps_L_fact > rps_L_gived) // Если больше чем надо то убавим оборотов
//         {
//             rps_new = rps_L_fact - (accel);
//             if (rps_new <= 0.01) // Если совсем уменьшили меньше минимума
//             {
//                 rps_new = 0;
//             }
//         }

//         Serial.print(" rps_L_gived= ");
//         Serial.print(rps_L_gived);
//         Serial.print(" rps_L_fact= ");
//         Serial.print(rps_L_fact);
//         Serial.print(" accel= ");
//         Serial.print(accel);
//         Serial.print(" rps_new L= ");
//         Serial.println(rps_new);

//         setTimeing_Step_L(rps_new);
//     }
// }
#endif