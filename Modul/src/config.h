#ifndef CONFIG_H
#define CONFIG_H


// Контакты для светодиодов:
const int PIN_LED_GREEN = 13; //Для красного светодиода перепаял
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


#define MAX_RADIUS 0.5      // Максимальный радиус поворота робота
#define FIX_SPEED 0.3  // Скорость при ручном управлении

hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
hw_timer_t *timer3 = NULL;

//*********************************************************************

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

#endif