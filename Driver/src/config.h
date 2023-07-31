#ifndef CONFIG_H
#define CONFIG_H

#include <Preferences.h> // Для сохранения значений во внутренней флеш памяти
Preferences preferences; // Создаем экземпляр для сохрания настроек и работы с флеш памятью

#include <EEPROM.h>    // Для сохранения значений во внутренней флеш памяти
#define EEPROM_SIZE 32 // define the number of bytes you want to access

// Контакты для светодиодов:
const int PIN_LED_GREEN = 15; //Для красного светодиода перепаял
//Нельзя использовать нулевой пин Не работает пин. При инициализации SPI_slave функция его занимает и дает всегда половину напряжения и он не реагирует на команды.
const int PIN_ANALIZ = 13;

uint64_t time_izmerenia = 0;      // Время в которое считываем данные из датчика
float delta_time_izmerenia = 0;   // Время между измерениями. передаем в ПИД регуляторы для расчетов
uint64_t pred_time_izmerenia = 0; // Предыдущее время измерения.

volatile bool flag_timer_1sec = false;
volatile bool flag_timer_10millisec = false;
volatile bool flag_timer_50millisec = false;
volatile bool flag_timer_60sec = false;
volatile int count_timer_10millisec = 0; //Счетчик для запуска обработки движения моторов в лупе по флагу
volatile int count_timer_50millisec = 0; //Счетчик для запуска каждые 50 милисекунд
volatile int count_timer_1sec = 0;       // Счетчик для запуска
volatile int count_timer_60sec = 0;      // Счетчик для запуска

int flag_newData = 0;        // Флаг что ест новые данные от обмена по шине SPI
int flag_newControlData = 0; // Флаг что есть новые данные по ручному управлению

int flag_setup = 0; // Флаг что функйия setup прошла успешно

int flag_servo_position_0 = 0; // Флаг команды встать в нчальное положение

#define MAX_RADIUS 0.5      // Максимальный радиус поворота робота
#define FIX_SPEED 0.3  // Скорость при ручном управлении

hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
hw_timer_t *timer3 = NULL;

//*********************************************************************
//Структура для углов наклонов
struct Struct_RPY
{
  float roll = 0;  // Крен в право  влево
  float pitch = 0; // Тангаж вверх или вних
  float yaw = 0;   // Поворот по часовой мом против часовой

  Struct_RPY &operator=(const Struct_RPY &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
  {
    roll = source.roll;
    pitch = source.pitch;
    yaw = source.yaw;
    return *this;
  }
};

//Структура для датчика напряжения INA219
struct Struct_INA
{
  float busVoltage_V = 0;
  float shuntVoltage_mV = 0;
  float current_mA = 0;
  float power_mW = 0;
};
//Структура одометрии
struct Struct_Odom
{
  float x = 0;      // Координата по Х
  float y = 0;      // Координата по Y
  float th = 0;     // Направление носа
  float vel_x = 0;  // Линейная скорость движения робота по оси X
  float vel_y = 0;  // Линейная скорость движения робота по оси Y
  float vel_th = 0; // Угловая скорость вращения робота

  Struct_Odom &operator=(const Struct_Odom &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
  {
    x = source.x;
    y = source.y;
    th = source.th;
    vel_x = source.vel_x;
    vel_y = source.vel_y;
    vel_th = source.vel_th;
    return *this;
  }
};

Struct_Odom g_odom_enc; // Одометрия по энкодерам
Struct_Odom g_odom_imu; // Одометрия по гироскопу и аксельрометру
float g_radius = 0;     // Радиус по которому движемся
// float g_napravl = 0;     // Направление поворота по часовой или против часовой
float g_speed = 0; // Скорость с которой движемся

// Структура получаемых данных от Data к контроллеру Driver
struct Struct_Data2Driver
{
  uint32_t id = 0;       // Номер команды по порядку
  uint32_t startStop;    // Стоим или двигаемся
  float radius = 0;      // Радиус по которому нужно двигаться
  float speed = 0;       // Скорость с которой нужно двигаться`
  float angle_camera;    // Угол камеры для шагового мотора камеры
  uint32_t led;          // Номер программы для светодиодов мигания
  uint32_t servo;        // Номер программы для сервомоторов движения рук
  int32_t posServoL;        // Позиция левого сервомотра
  int32_t posServoR;        // ПИзиция правого сервомотра
  int32_t timeServoL;       // Время за которое левый мотор должен прити в задаваемую позицию
  int32_t timeServoR;       // Время за которое правый мотор должен прити в задаваемую позицию
  uint32_t cheksum = 0;  // Контрольная сумма данных в структуре
};

Struct_Data2Driver Data2Driver_receive;                         // Экземпляр структуры получаемых данных
const int size_structura_receive = sizeof(Data2Driver_receive); // Размер структуры с данными которые получаем

//Структура в которой все главные переменные передаюся на высокий уровень
struct Struct_Driver2Data
{
  uint32_t id = 0;      // id команды
  Struct_Odom odom_enc; // Одометрия по энкодерам
  Struct_Odom odom_imu; // Одометрия по гироскопу и аксельрометру
  float odom_L = 0;     // Пройденный путь левым колесом
  float odom_R = 0;     // Пройденный путь правым колесом
  float speed_L = 0;    // Скорость левого колеса
  float speed_R = 0;    // Скорость правого колеса
  Struct_RPY bno055;    // Данные с датчика BNO055
  Struct_INA ina;       // Данные с датчика INA219

  uint32_t connect_flag; // Флаг связи с пультом ручного управления
  uint32_t startStop;    // Стоим или двигаемся
  float radius = 0;      // Радиус по которому нужно двигаться
  float speed = 0;       // Скорость с которой нужно двигаться
  float angle_camera;    // Угол камеры для шагового мотора камеры
  uint32_t led;          // Номер программы для светодиодов мигания
  uint32_t servo;        // Номер программы для сервомоторов движения рук
  int32_t posServoL;        // Позиция левого сервомотра
  int32_t posServoR;        // ПИзиция правого сервомотра

  uint32_t status_wifi; // Статус запущен ли вайфай или выключен

  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Driver2Data Driver2Data_send;                                                                                    //Тело робота. тут все переменные его характеризующие на низком уровне
const int size_structura_send = sizeof(Driver2Data_send);                                                               // Размер структуры с данными которые передаем
const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше

//Функция возвращает контрольную сумму структуры без последних 4 байтов
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

/*

//Структура точки
struct Struct_Point
{
  float x = 0;                                        // Координата по Х
  float y = 0;                                        // Координата по Y
  float z = 0;                                        // Координата по Z
  Struct_Point &operator=(const Struct_Point &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
  {
    x = source.x;
    y = source.y;
    z = source.z;
    return *this;
  }
};
//Структура вектора
struct Struct_Vector3
{
  float x = 0;                                            // Координата по Х
  float y = 0;                                            // Координата по Y
  float z = 0;                                            // Координата по Z
  Struct_Vector3 &operator=(const Struct_Vector3 &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
  {
    x = source.x;
    y = source.y;
    z = source.z;
    return *this;
  }
};
//Структура ориентации
struct Struct_Quaternion
{
  float x = 0;                                                  // Координата по Х
  float y = 0;                                                  // Координата по Y
  float z = 0;                                                  // Координата по Z
  float w = 0;                                                  // Координата по W
  Struct_Quaternion &operator=(const Struct_Quaternion &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
  {
    x = source.x;
    y = source.y;
    z = source.z;
    w = source.w;
    return *this;
  }
};
//Структура позиции
struct Struct_Pose
{
  Struct_Point position;         //  Позиция
  Struct_Quaternion orientation; // Ориентация в позиции
};
//Структура движения
struct Struct_Twist
{
  Struct_Vector3 linear;  //  Позиция
  Struct_Vector3 angular; // Ориентация в позиции
};

//Структура движения
struct Struct_Odometry
{
  String child_frame_id = "base_link"; //  СИстема координат
  Struct_Pose pose;                    // позиция
  Struct_Twist twist;                  // движение
};
*/

#endif