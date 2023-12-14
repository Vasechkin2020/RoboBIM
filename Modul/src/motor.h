#ifndef MOTOR_H
#define MOTOR_H

//---------------------------------------------------------------------------------------
#define PIN_En 25 //
//---------------------------------------------------------------------------------------
#define PIN_L_Step 26 //
#define PIN_L_Dir 27  //
//---------------------------------------------------------------------------------------
#define PIN_R_Step 32 //
#define PIN_R_Dir 33  //

#define DIAMETR 66 // Влияет на правильность длинны через расчет скорости
#define RADIUS (DIAMETR / 2)
#define PERIMETR (DIAMETR * PI)
#define DISTANCE_WHEELS 0.186 // Растояние между колесами робота. подобрал экспериментально Влияет на правильность круга
#define MAX_SPEED 0.6


#define ACCELERATION_UP 0.5    // Ускорение машинки константа в метрах в секунду
#define ACCELERATION_DOWN -0.3 // Торможение машинки константа в метрах в секунду
//---------------------------------------------------------------------------------------

bool flag_motor_L_EN = 0;             //Флаг работает или нет Левый мотор
bool flag_motor_R_EN = 0;             //Флаг работает или нет Правый мотор
bool flag_command_stop_motor = false; // Флаг что команда остановиться была и нужно отследить время чтобы их отключить

uint64_t time_command_stop_motor = 0; // Время в которое дали команду остановиться моторам

int microStep;      // Число микрошагов
int Timeing_Step_L; // Число тактов для таймера через которое нужно дать новый имульс мотору
int Timeing_Step_R; // Число тактов для таймера через которое нужно дать новый имульс мотору

//Одометрия
float speed_L_fact; //Скорость на моторе фактически установленная
float speed_R_fact; //Скорость на моторе фактически установленная

float odom_way_L = 0; //Пройденный путь левым колесом с учетом направления вращения
float odom_way_R = 0; //Пройденный путь правым колесом с учетом направления вращения
float odom_way_C = 0; //Пройденный путь правым колесом с учетом направления вращения

int16_t odom_impuls_L = 0; // Количество ипульсов переданных колесу с учетом направления вращения
int16_t odom_impuls_R = 0; // Количество ипульсов переданных колесу с учетом направления вращения

int8_t odom_dir_L = 0; //Направление вращения левого колеса в данный момент 1- вперед, -1 - назад, 0 стоим на месте
int8_t odom_dir_R = 0; //Направление вращения правого колеса в данный момент 1- вперед, -1 - назад, 0 стоим на месте

uint16_t time_speed = 0;       // время в течении которого надо двигаться
uint64_t time_speed_start = 0; // время в который момент начали движение
bool time_speed_flag = 0;      // Флаг что надо отслеживать движение по времени

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
//Функция исполняемая по прерыванию по таймеру 1 ЛЕВЫЙ МОТОР
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
//Функция исполняемая по прерыванию по таймеру 2 ПРАВЫЙ МОТОР
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

// Функция устанавлявающая скорость вращения на левом моторе
void setSpeed_L(float _speed)
{
    // printf(" setSpeed_L= %f \n", _speed);
    if (_speed == 0)
    {
        flag_motor_L_EN = false; //Флаг ставим что мотор не работает, просто перестаем делать импульсы
    }
    else
    {
        flag_command_stop_motor = false; //Отменяем выключение драйверов с задержкой
        digitalWrite(PIN_En, 0);         //Включаем драйвера
        if (_speed > MAX_SPEED)
            _speed = MAX_SPEED;
        if (_speed < -MAX_SPEED)
            _speed = -MAX_SPEED;
        //Задаем направление вращения
        if (_speed < 0)
        {
            digitalWrite(PIN_L_Dir, 0);
            odom_dir_L = -1; // Тут назначаем куда вращаемся для одометрии для левого и праввого колеса может быть надо по разному
        }
        if (_speed > 0)
        {
            digitalWrite(PIN_L_Dir, 1);
            odom_dir_L = 1; // Тут назначаем куда вращаемся для одометрии для левого и праввого колеса может быть надо по разному
        }

        float step_za_sec = ((abs(_speed) * 1000) / PERIMETR * 360 / 1.8); //Скорость приводим в милиметры и делим на периметр колеса получаем обороты их умножаем на градусы и делим на градус на 1 шаг получаем нужное число полных шагов для такой скорости за секунду
        //Serial.print(" step_za_sec= ");
        //Serial.println (step_za_sec);

        // Микросекунды в секунде делим на число шагов которые надо успеть сделать за секунду и делим на микрошаги делим на микросекунды за 1 шаг с учетом предделителя таймера
        Timeing_Step_L = (float)1000000 / step_za_sec / microStep / 1.0; // Таймер по 1 микросекунде
        flag_motor_L_EN = true;                                          //Взводим флаг разрешающий делать импульсы в обработчике прерывания
    }
    speed_L_fact = _speed; //Запоминаем скорость с которой установили вращаться мотору
}

// Функция устанавлявающая скорость вращения на правом моторе
void setSpeed_R(float _speed)
{
    // printf(" setSpeed_R= %f \n", _speed);
    if (_speed == 0)
    {
        flag_motor_R_EN = false; //Флаг ставим что мотор не работает это чтобы импульс не подавался
    }
    else
    {
        flag_command_stop_motor = false; //Отменяем выключение драйверов с задержкой
        digitalWrite(PIN_En, 0);         //Включаем драйвера
        if (_speed > MAX_SPEED)
            _speed = MAX_SPEED;
        if (_speed < -MAX_SPEED)
            _speed = -MAX_SPEED;
        //Задаем направление вращения
        if (_speed < 0)
        {
            digitalWrite(PIN_R_Dir, 0);
            odom_dir_R = -1; // Тут назначаем куда вращаемся для одометрии для левого и правого колеса может быть надо по разному
        }
        if (_speed > 0)
        {
            digitalWrite(PIN_R_Dir, 1);
            odom_dir_R = 1; // Тут назначаем куда вращаемся для одометрии для левого и правого колеса может быть надо по разному
        }

        float step_za_sec = ((abs(_speed) * 1000) / PERIMETR * 360 / 1.8); //Скорость приводим в милиметры и делим на периметр колеса получаем обороты их умножаем на градусы и делим на градус на 1 шаг получаем нужное число полных шагов для такой скорости за секунду
        //Serial.print(" step_za_sec= ");
        //Serial.println(step_za_sec);

        // Микросекунды в секунде делим на число шагов которые надо успеть сделать за секунду и делим на микрошаги делим на микросекунды за 1 шаг с учетом предделителя таймера
        Timeing_Step_R = (float)1000000 / step_za_sec / microStep / 1.0; // Таймер по 1 микросекунде
        flag_motor_R_EN = true;                                          //Взводим флаг разрешающий делать импульсы в обработчике прерывания
    }
    speed_R_fact = _speed; //Запоминаем скорость с которой установили вращаться мотору
}

//Остановка моторов  и установкка времени задержки выключения драйверов
void stopMotor()
{
    setSpeed_L(0);
    setSpeed_R(0);
    g_speed = 0;  // Для одометрии скорость ставим 0
    g_radius = 0; // Для одометрии радиус ставим 0
    //Отсрочка выключения драйверов чтобы не грелись и электричество не потребляли
    time_command_stop_motor = millis();
    flag_command_stop_motor = true; //Взводим флаг выключение драйверов с задержкой в таймере
}

//Инициализация таймера 1. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void initTimer_1()
{
    timer1 = timerBegin(1, 80, true);              // Номер таймера, делитель(прескаллер), Считать вверх, прибавлять (true)  Частота базового сигнала 80  Мега герц, значит будет 1 микросекунда
    timerAttachInterrupt(timer1, &onTimer1, true); // Какой таймер используем, какую функцию вызываем,  тип прерывания  edge или level interrupts
    timerAlarmWrite(timer1, 100000, true);         // Какой таймер, до скольки считаем , сбрасываем ли счетчик при срабатывании. 1000 микросекунд эти 1 милисекунда
    timerAlarmEnable(timer1);                      // Запускаем таймер
}
//Инициализация таймера 1. Всего 4 таймера вроде от 0 до 3 //https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void initTimer_2()
{
    timer2 = timerBegin(2, 80, true);              // Номер таймера, делитель(прескаллер), Считать вверх, прибавлять (true)  Частота базового сигнала 80  Мега герц, значит будет 1 микросекунда
    timerAttachInterrupt(timer2, &onTimer2, true); // Какой таймер используем, какую функцию вызываем,  тип прерывания  edge или level interrupts
    timerAlarmWrite(timer2, 100000, true);         // Какой таймер, до скольки считаем , сбрасываем ли счетчик при срабатывании. 1000 микросекунд эти 1 милисекунда
    timerAlarmEnable(timer2);                      // Запускаем таймер
}

void initMotor()
{
    //**************************************************************   Мотор на колеса
    microStep = 8; // Так распаяно на плате для драйверов 2208 и 2209
    printf("Init StepMotor1. Set microstep = %i \n", microStep);
    Serial.print("Init StepMotor2. Set microstep = ");
    Serial.println(microStep);

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

    flag_motor_L_EN = false;         //Флаг ставим что мотор не работает, просто перестаем делать импульсы
    flag_motor_R_EN = false;         //Флаг ставим что мотор не работает, просто перестаем делать импульсы
    flag_command_stop_motor = false; //Отменяем выключение драйверов с задержкой

    initTimer_1(); //Таймер на 1 мотор
    initTimer_2(); //Таймер на 2 мотор
    // setSpeed_R(0.5);
    // setSpeed_L(0.5);
    // Serial.println("Test Motor");
    // while (1);
}

//Функция задержки выключения моторов после команды стоп на 1 секунду для экономии энергии и чтобы не грелись
void delayMotor()
{
    // Отслеживания времени отключить драйвера на моторах чтобы не грелись и не потребляли энергию
    if (flag_command_stop_motor)
    {
        if (millis() - time_command_stop_motor > 1000) // Если с момента остановки моторов прошло более 1000 милисекунд (1 секунда) и флаг не изменился другой командой
        {
            flag_command_stop_motor = false; //Заканчиваем отслеживать
            digitalWrite(PIN_En, 1);         //Выключаем драйвера
        }
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

    float way_L = ((PI * (RADIUS / 1000.0)) / 180.0) * gradusL; // По формуле длинны дуги находим пройденный путь колесом Радиус приводим в метры так как он указан в мм
    float way_R = ((PI * (RADIUS / 1000.0)) / 180.0) * gradusR; // По формуле длинны дуги находим пройденный путь колесом
    float way_C = (way_L + way_R) / 2.0;                        // Путь средней точки Center
    odom_way_L += way_L;                                        // суммируем
    odom_way_R += way_R;                                        // суммируем
    odom_way_C += way_C;                                        // суммируем

    //printf("dt= %f odom_way_L= %f  odom_way_R= %f ", dt, odom_way_L, odom_way_R);
    // printf("odom_way_C= %f ", odom_way_C);

    //-----------------------------------
    float angle_pov;
    //Находим угловую скорость вращения (поворота) робота
    if (g_radius == 0) // Если двигаемся прямо то
    {
        angle_pov = 0; // Угол поворота равен нулю
    }
    else
    {
        angle_pov = way_C / -g_radius; // Делим пройденный оп дуге путь на радиус движения получаем угол поворота в радианах
        angle_pov_sum += angle_pov * 180.0 / PI;
    }
    //Находим угловую скорость поворота в радианах в секунду
    g_odom_enc.vel_th = angle_pov / dt; //  Вычисляем радианы в секунду получаем угловую скорость
    //---------------------------------------
    // Находим линейные скорости из моего вектора скорости. Я задаю общую скорость движения (длинна вектора), ее надо разложить на проекции по осям x y. Это будут линейные скорости
    g_odom_enc.vel_x = g_speed * cos(g_odom_enc.th); // Проекция моей скорости на ось X получаем линейную скорость по оси
    g_odom_enc.vel_y = g_speed * sin(g_odom_enc.th); // Проекция моей скорости на ось Y получаем линейную скорость по оси

    //printf("g_speed= %.2f  g_radius= %.2f angle_pov= %f  angle_pov_sum= %f vel_x= %.2f  vel_y= %.2f  vel_th= %f ", g_speed, g_radius, angle_pov, angle_pov_sum, g_odom_enc.vel_x, g_odom_enc.vel_y, g_odom_enc.vel_th);

    // Находим смещение по глобальной карте по осям Х и Y c помощью матрицы вращения. Она вычисляет смешения по осям на нужный угол
    //float delta_x = (g_odom_enc.vel_x * cos(g_odom_enc.th) - g_odom_enc.vel_y * sin(g_odom_enc.th)) * dt;
    //float delta_y = (g_odom_enc.vel_x * sin(g_odom_enc.th) + g_odom_enc.vel_y * cos(g_odom_enc.th)) * dt;

    float delta_x = (g_odom_enc.vel_x) * dt;
    float delta_y = (g_odom_enc.vel_y) * dt;
    float delta_th = g_odom_enc.vel_th * dt;

    // Меняем координаты и угол на основе вычислений
    g_odom_enc.x += delta_x;   //Вычисляем координаты
    g_odom_enc.y += delta_y;   //Вычисляем координаты
    g_odom_enc.th += delta_th; // Прибавляем к текущему углу и получаем новый угол куда смотрит наш робот

    //printf("x= %.2f y= %.2f th= %.3f  time= %u \n", g_odom_enc.x, g_odom_enc.y, g_odom_enc.th, millis());
}


// Установка скорости моторов в зависимости от радиуса и направления
void set_speed_master_slave(float speed_, float radius_)
{
    g_radius = radius_; 
    g_speed = speed_; 

    float k2 = 0;
    if (g_radius != 0) // Вычисляем коефициент перобразования иначе он остается равен 0
    {
        k2 = (DISTANCE_WHEELS / 2.0) / abs(g_radius); //Находим долю какую составляем растояние половинки между колесами тела и радиусом поворота.
        //printf("k2= %f \n", k2);
    }
    float internal_speed = g_speed * (1 - k2);
    float external_speed = g_speed * (1 + k2);
    //printf(" internal_speed= %f  center_speed= %f external_speed= %f \n", internal_speed, g_speed, external_speed);

    // Определяем направление движения
    if (radius_ == 0) //Едем по прямой
    {
        setSpeed_L(g_speed);
        setSpeed_R(g_speed);
    }
    if (radius_ < 0) //  едем против часовой стрелки, поворачиваем налево
    {
        setSpeed_L(internal_speed); //Левое едет медленней
        setSpeed_R(external_speed); // Правое едет быстрее
    }
    if (radius_ > 0) // По часовой стрелке направо
    {
        setSpeed_L(external_speed); //Левое едет быстрее
        setSpeed_R(internal_speed); // Правое едет медленно
    }
}
// Функция отслеживает время движения. Движемся только опредленное времяи потом если нет новой команды останавливаемся
void movementTime()
{
    // Отслеживания движения по времени
    if (time_speed_flag)
    {
        if (millis() - time_speed_start > time_speed) //
        {
            time_speed_flag = false; //Заканчиваем отслеживать
            stopMotor();
            time_speed_start = time_speed = 0;
        }
    }
}
// Установка скорости движения в течении заданного времени в милисекундах с заданным радиусом
void setSpeed_time(float speed_, float radius_, float time_)
{
    time_speed = time_; //Запоминаем время
    time_speed_start = millis();
    time_speed_flag = true;                  //Взводим флаг
    set_speed_master_slave(speed_, radius_); // Установка скорости моторов в зависимости от радиуса, прибавляем половину от растояния между колесами так как считаем от внешнего колеса, может и не правильно
}


#endif