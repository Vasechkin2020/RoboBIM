#ifndef CONFIG_H
#define CONFIG_H

#define NUM_LEDS 43 // Колличество светодиодов всего
#define REDUCTOR 10 // Коеффициент редуктора на колесах

#include <EEPROM.h>    // Для сохранения значений во внутренней флеш памяти
#define EEPROM_SIZE 32 // define the number of bytes you want to access

// Контакты для светодиодов:
#define PIN_LED_GREEN 4 // Совпадает с драйверами на платформу.
// Нельзя использовать нулевой пин Не работает пин. При инициализации SPI_slave функция его занимает и дает всегда половину напряжения и он не реагирует на команды.
// #define PIN_ANALIZ 27 //Совпадает с драйвером под платформу, закоментить изменить после отладки

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

#define Addr_TCA9548A 0x70     // Адреc платы мультиплексора шины I2C
#define multi_line_VL53L0X_L 1 // На какой линии мультиплексора находится датчик VL53L0X
#define multi_line_VL53L0X_R 0 // На какой линии мультиплексора находится датчик VL53L0X
#define multi_line_BNO2 2      // На какой линии мультиплексора находится датчик BNO055

//****************************************************************************************************************************************************
// Структура по обратной связи по обмену по шине SPI

typedef struct // Структура для хранения 3-х координат
{
  float x;
  float y;
  float z;
} SXyz;

struct SSpi
{
  uint32_t all = 0;
  uint32_t bed = 0;
};
//****************************************************************************************************************************************************
// Структура управления движением
struct SControl
{
  float speedL = 0; // Скорость с которой нужно двигаться в оборотах/секунда !!!!!!!!!
  float speedR = 0; // Скорость с которой нужно двигаться в оборотах/секунда !!!!!!!!!!!!!!!!
};
// Структура управления Светодиодами
struct SLed
{
  uint8_t led[NUM_LEDS]; // Маасив через который управляем светодиодами
};
// Структура получаемых данных от Data к контроллеру Driver
struct Struct_Data2Driver
{
  uint32_t id = 0;      // Номер команды по порядку
  SControl control;     // Структура управления машиной
  SLed led;             // Управление светодиодами
  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Data2Driver Data2Driver_receive;                         // Экземпляр структуры получаемых данных
const int size_structura_receive = sizeof(Data2Driver_receive); // Размер структуры с данными которые получаем

//*****************************************************************************************************************************************************
// Структура сосдержит всю информацию по мотору на основании данных энкодера
struct SMotor
{
  uint32_t statusDriver = 1; // Статус драйвера.Включен или выключен.Удерживаются колосеса или свободно катаются 1-выключен
  float rpsEncodL = 0;       // Реальная скорость вращения по енкодерам( обороты в секунду)
  float rpsEncodR = 0;       // Реальная скорость вращения по енкодерам( обороты в секунду)
};

struct SMpu // Структура с данными со всех датчиков, отправляем наверх
{
  int32_t status = 0; // статус состояния
  float rate;         // частота работы датчика
  SXyz angleEuler;
  SXyz linear;
};
struct SIcm // Структура с данными со всех датчиков, отправляем наверх
{
  int32_t status = 0; // статус состояния
  float rate;         // частота работы датчика
  SXyz accel;         // акселерометр
  SXyz gyro;          // гиороскоп
};

// Структура состояния датчика расстония
struct SSensor
{
  int32_t status = 0; // статус состояния
  float distance = 0; // расстояние до препятствия
};
// Отдельные структуры для каждой сущности
SMotor motor;
SMpu bno055;   // Данные с датчика BNO055
SIcm icm20948; // Данные с датчика icm20948
SSensor laserL;
SSensor laserR;
SSensor uzi;
SSpi spi; // Структура по состоянию обмена по шине SPI

// Структура в которой все главные переменные передаюся на высокий уровень
struct Struct_Driver2Data
{
  uint32_t id = 0; // id команды
  SMotor motor;
  SIcm icm; // Данные с датчика ICM20948
  SSensor laserL;
  SSensor laserR;
  SSensor uzi;
  SSpi spi;             // Структура по состоянию обмена по шине SPI
  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

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