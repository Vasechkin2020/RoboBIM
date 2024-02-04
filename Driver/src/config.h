#ifndef CONFIG_H
#define CONFIG_H

#include <EEPROM.h>    // Для сохранения значений во внутренней флеш памяти
#define EEPROM_SIZE 32 // define the number of bytes you want to access

// Контакты для светодиодов:
#define PIN_LED_GREEN 4  // Совпадает с драйверами на платформу.
// Нельзя использовать нулевой пин Не работает пин. При инициализации SPI_slave функция его занимает и дает всегда половину напряжения и он не реагирует на команды.
// #define PIN_ANALIZ 27 //Совпадает с драйвером под платформу, закоментить изменить после отладки

uint64_t time_izmerenia = 0;      // Время в которое считываем данные из датчика
float delta_time_izmerenia = 0;   // Время между измерениями. передаем в ПИД регуляторы для расчетов
uint64_t pred_time_izmerenia = 0; // Предыдущее время измерения.

volatile bool flag_timer_1sec = false;
volatile bool flag_timer_10millisec = false;
volatile bool flag_timer_50millisec = false;
volatile int count_timer_10millisec = 0; // Счетчик для запуска обработки движения моторов в лупе по флагу
volatile int count_timer_50millisec = 0; // Счетчик для запуска каждые 50 милисекунд
volatile int count_timer_1sec = 0;       // Счетчик для запуска

hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
hw_timer_t *timer3 = NULL;

#define Addr_TCA9548A 0x70 // Адреc платы мультиплексора шины I2C
#define multi_line_BNO 2       // На какой линии мультиплексора находится датчик BNO055
#define multi_line_VL53L0X_L 0 // На какой линии мультиплексора находится датчик VL53L0X
#define multi_line_VL53L0X_R 1 // На какой линии мультиплексора находится датчик VL53L0X

//****************************************************************************************************************************************************
//Структура по обратной связи по обмену по шине SPI
struct  SSpi
{
  uint32_t all = 0;
  uint32_t bed = 0;
};
// Структура управления движением
struct SControl
{
  float speedL = 0;        // Скорость с которой нужно двигаться`
  float speedR = 0;        // Скорость с которой нужно двигаться`
};
// Структура управления Светодиодами
struct SLed
{
  int32_t num_program = 0; // Номер программы для светодиодов мигания
};
// Структура получаемых данных от Data к контроллеру Driver
struct Struct_Data2Driver
{
  uint32_t id = 0;        // Номер команды по порядку
  SControl control; // Структура управления машиной
  SLed led;         // Управление светодиодами
  uint32_t cheksum = 0;   // Контрольная сумма данных в структуре
};

Struct_Data2Driver Data2Driver_receive;                         // Экземпляр структуры получаемых данных
const int size_structura_receive = sizeof(Data2Driver_receive); // Размер структуры с данными которые получаем

//*****************************************************************************************************************************************************
// Структура сосдержит всю информацию по мотору на основании данных энкодера
struct SEncoder
{
  float rpsSet = 0; // Заданная скорость вращения ( обороты в секунду)
  float rpsEncod = 0; // Реальная скорость вращения по енкодерам( обороты в секунду)
};
// Структура сосдержит общую информацию по машинке
struct SCar
{
  float speedSetL = 0;  // Скорость которую задали в функции (метры в секунду)
  float speedSetR = 0;  // Скорость которую задали в функции (метры в секунду)
  float speedEncod = 0;  // Текущая скорость движения (метры в секунду)
  float way = 0; // Пройденный путь в метрах
};
// Структура для углов наклонов
struct Struct_IMU
{
  float roll = 0;   // Крен в право  влево
  float pitch = 0;  // Тангаж вверх или вних
  float yaw = 0;    // Поворот по часовой мом против часовой
  float x = 0;      // Координата по Х
  float y = 0;      // Координата по Y
  float th = 0;     // Направление носа
  float vel_x = 0;  // Линейная скорость движения робота по оси X
  float vel_y = 0;  // Линейная скорость движения робота по оси Y
  float vel_th = 0; // Угловая скорость вращения робота
};

// Структура состояния датчика расстония
struct SSensor
{
  float distance; // расстояние до препятствия
};
// Структура в которой все главные переменные передаюся на высокий уровень
struct Struct_Driver2Data
{
  uint32_t id = 0; // id команды
  SCar car;
  SEncoder motorLeft;
  SEncoder motorRight;
  Struct_IMU bno055;    // Данные с датчика BNO055
  SSensor lazer1;
  SSensor lazer2;
  SSensor uzi1;
  SSpi spi;            // Структура по состоянию обмена по шине SPI
  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

//Отдельные структуры для каждой сущности
SCar car; // Данные в целом по машинке
SEncoder motorLeft;
SEncoder motorRight;
Struct_IMU bno055;    // Данные с датчика BNO055
SSensor lazer1;
SSensor lazer2;
SSensor uzi;
SSpi spi;            // Структура по состоянию обмена по шине SPI

Struct_Driver2Data Driver2Data_send;                                                                                           // Тело робота. тут все переменные его характеризующие на низком уровне
const int size_structura_send = sizeof(Driver2Data_send);                                                                      // Размер структуры с данными которые передаем
const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше

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

/*
// // Структура одометрии
// struct Struct_Odom
// {
//   float x = 0;      // Координата по Х
//   float y = 0;      // Координата по Y
//   float th = 0;     // Направление носа
//   float vel_x = 0;  // Линейная скорость движения робота по оси X
//   float vel_y = 0;  // Линейная скорость движения робота по оси Y
//   float vel_th = 0; // Угловая скорость вращения робота

//   Struct_Odom &operator=(const Struct_Odom &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
//   {
//     x = source.x;
//     y = source.y;
//     th = source.th;
//     vel_x = source.vel_x;
//     vel_y = source.vel_y;
//     vel_th = source.vel_th;
//     return *this;
//   }
// };

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