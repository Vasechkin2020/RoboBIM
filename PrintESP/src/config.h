#ifndef CONFIG_H
#define CONFIG_H

// Контакты для светодиодов:
const int PIN_LED_RED = 33; //Для красного светодиода перепаял
//Нельзя использовать нулевой пин Не работает пин. При инициализации SPI_slave функция его занимает и дает всегда половину напряжения и он не реагирует на команды.
const int PIN_LED_BLUE = 32;
const int PIN_ANALIZ = 4;

float filtr_My(float old_, float new_, float ves_new_); // Просто предолбявление функции. Реализация дальше. что-бы компилятор не ругался

volatile bool flag_timer_1sec = false;
volatile bool flag_timer_10millisec = false;
volatile bool flag_timer_50millisec = false;
volatile bool flag_timer_60sec = false;
volatile int count_timer_10millisec = 0; //Счетчик для запуска обработки движения моторов в лупе по флагу
volatile int count_timer_50millisec = 0; //Счетчик для запуска каждые 50 милисекунд
volatile int count_timer_1sec = 0;       // Счетчик для запуска
volatile int count_timer_60sec = 0;      // Счетчик для запуска

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


//Структура в которой все собранные данные передаются из Iot к Data
struct Struct_Iot2Data
{
  uint32_t id = 0;            // id команды
  uint32_t cheksum = 0;       // Контрольная сумма данных в структуре
};

Struct_Iot2Data Iot2Data_send;                                                                                                         //Тело робота. тут все переменные его характеризующие на низком уровне
const int size_structura_send = sizeof(Iot2Data_send);                                                                             // Размер структуры с данными которые передаем

// Структура получаемых данных из Data к Iot
struct Struct_Data2Iot
{

  uint32_t id = 0;            // Id команды
  float odom_L = 0;     // Пройденный путь левым колесом
  float odom_R = 0;     // Пройденный путь правым колесом
  float speed_L = 0;    // Скорость левого колеса
  float speed_R = 0;    // Скорость правого колеса
  Struct_RPY bno055;    // Данные с датчика BNO055

  uint32_t connect_flag; // Флаг связи с пультом ручного управления
  uint32_t startStop;    // Стоим или двигаемся
  float radius = 0;      // Радиус по которому нужно двигаться
  float speed = 0;       // Скорость с которой нужно двигаться
  float angle_camera;    // Угол камеры для шагового мотора камеры
  uint32_t led;          // Номер программы для светодиодов мигания
  uint32_t servo;        // Номер программы для сервомоторов движения рук
  int32_t posServoL;        // Позиция левого сервомотра
  int32_t posServoR;        // ПИзиция правого сервомотра


  uint32_t cheksum = 0; // Контрольная сумма данных в структуре


  // uint32_t id = 0;          // Номер команды по порядку
  // int32_t command_body = 0; // Команда для выполнения
  // float radius = 0;         // Радиус по которому нужно двигаться
  // float speed = 0;          // Скорость которую нужно установить
  // float motor_video_angle;  // Угол для шагового мотора камеры
  // uint32_t cheksum = 0;     // Контрольная сумма данных в структуре
};

Struct_Data2Iot Data2Iot_receive;                           // Экземпляр структуры получаемых данных
const int size_structura_receive = sizeof(Data2Iot_receive); // Размер структуры с данными которые получаем

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