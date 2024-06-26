#ifndef CONFIG_H
#define CONFIG_H

// Контакты для светодиодов:
#define PIN_LED 4 // Для светодиода // !!! Нельзя использовать нулевой пин Не работает пин. При инициализации SPI_slave функция его занимает и дает всегда половину напряжения и он не реагирует на команды.

volatile bool flag_timer_1sec = false;
volatile bool flag_timer_10millisec = false;
volatile bool flag_timer_50millisec = false;
volatile int count_timer_10millisec = 0; // Счетчик для запуска обработки движения моторов в лупе по флагу
volatile int count_timer_50millisec = 0; // Счетчик для запуска каждые 50 милисекунд
volatile int count_timer_1sec = 0;       // Счетчик для запуска

u_int64_t timeSpi = 0; //Время когда пришла команда по SPI

hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;

//*********************************************************************
struct motorStruct // Структура для локального управления и сбора данных по моторам
{
  int32_t status = 0;      // Передаются импульсы на мотор или нет в данный момент, вращается или нет
  int32_t position = 0;    // Текущая позиция в импульсах
  int32_t destination = 0; // Цель назначение в позиции в импульсах
  int8_t dir = 0;          // Направление вращения мотора 1 - по часовой 0 - против часовой
  int8_t dir_pin = 0;      // Пин определяющий направление вращения
  int8_t step_pin = 0;     // Пин определяющий импульс
  int8_t micric_pin = 0;   // Пин определяющий концевик
};
motorStruct motor[4]; // Все локальные данные по моторам

//*********************************************************************
struct SControlLaser
{
  uint32_t mode = 0; // Текущий режим работы 0 - отключить датчики 1 - режим одновременного измерения
};
struct SControlMotor
{
  uint32_t mode = 0;     // Текущий режим работы 0 - режим колибровки концевиков 1 - обычное управление моторами по углу
  float angle[4];        // Углы в которые нужно повернультя в локальной системе
  int32_t numPillar[4]; // Номер столба до которого измеряем расстояние
};

// Структура получаемых данных от Data к контроллеру Modul
struct Struct_Data2Modul
{
  uint32_t id = 0;            // Номер команды по порядку
  SControlMotor controlMotor; // Управление моторами
  SControlLaser controlLaser; // Управление лазерами
  uint32_t cheksum = 0;       // Контрольная сумма данных в структуре
};

Struct_Data2Modul Data2Modul_receive; // Экземпляр структуры получаемых данных

const int size_structura_receive = sizeof(Data2Modul_receive); // Размер структуры с данными которые получаем

//*********************************************************************
// Структура по состоянию лидаров которая передается на верхний уровень
struct SLaserSend
{
  uint32_t status = 0;        // Статус датчика или ошибки
  float distance = 0;      // Последнее измерение
  uint32_t signalQuality = 0; // Качество сигнала
  float angle = 0;            // Положение при последнем измерении
  int32_t numPillar = -1;     // Номер столба до которого измерили расстояние
};
SLaserSend laser[4]; // Все локальные данные по лазерам

struct SMotorSend // Структура которая передается на верхний уровень
{
  int32_t status = 0;    // Передаются импульсы на мотор или нет в данный момент, вращается или нет
  float position = 0;    // Текущая позиция в градусах
  float destination = 0; // Цель назначение в позиции в градусах
};
// Структура по обратной связи по обмену по шине SPI
struct SSpi
{
  uint32_t all = 0;
  uint32_t bed = 0;
};
SSpi spi; // Переменная где все данные по обмену

// Структура в которой все главные переменные передаюся на высокий уровень от Modul к Data
struct Struct_Modul2Data
{
  uint32_t id = 0; // id команды

  uint32_t pinMotorEn = 0;      // Стутус пина управления драйвером моторов, включен драйвер или нет
  SMotorSend motor[4];          // Структура по состоянию моторов
  SLaserSend laser[4];          // Структура по состоянию лазеров
  uint32_t statusDataLaser = 0; // Статус обновления данных с лазерных датчиков
  uint32_t micric[4];           // Структура по состоянию концевиков
  SSpi spi;                     // Структура по состоянию обмена по шине SPI

  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Modul2Data Modul2Data_send;                                                                                             // Тут все переменные его характеризующие на низком уровне
const int size_structura_send = sizeof(Modul2Data_send);                                                                       // Размер структуры с данными которые передаем
const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше

#endif