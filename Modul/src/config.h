#ifndef CONFIG_H
#define CONFIG_H

// Контакты для светодиодов:
const int PIN_LED = 22; // Для светодиода // !!! Нельзя использовать нулевой пин Не работает пин. При инициализации SPI_slave функция его занимает и дает всегда половину напряжения и он не реагирует на команды.

volatile bool flag_timer_1sec = false;
volatile bool flag_timer_10millisec = false;
volatile bool flag_timer_50millisec = false;
volatile int count_timer_10millisec = 0; // Счетчик для запуска обработки движения моторов в лупе по флагу
volatile int count_timer_50millisec = 0; // Счетчик для запуска каждые 50 милисекунд
volatile int count_timer_1sec = 0;       // Счетчик для запуска

hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;

//*********************************************************************
//Структура по обратной связи по обмену по шине SPI
struct  spiStruct
{
  uint32_t all = 0;
  uint32_t bed = 0;
};
spiStruct spi; // Переменная где все данные по обмену

//*********************************************************************
// Структура по состоянию лидаров локальная
struct lidarStruct
{
  uint32_t status = 0; // Статус датчика или ошибки
  float distance = 0;  // Последнее измерение
  float angle = 0;     // Положение при последнем измерении
};

lidarStruct lidar[4]; // Все локальные данные по дальномерам

// Структура по состоянию лидаров которая передается на верхний уровень
struct lidarStructSend
{
  uint32_t status = 0; // Статус датчика или ошибки
  float distance = 0;  // Последнее измерение
  float angle = 0;     // Положение при последнем измерении
};

//*********************************************************************
struct motorStruct // Структура для локального управления и сбора данных по моторам
{
  int32_t status = 0;          // Передаются импульсы на мотор или нет в данный момент, вращается или нет
  int32_t position = 0;        // Текущая позиция в импульсах
  int32_t destination = 0;     // Цель назначение в позиции в импульсах
  int8_t dir = 0;              // Направление вращения мотора 1 - по часовой 0 - против часовой
  int8_t dir_pin = 0;          // Пин определяющий направление вращения
  int8_t step_pin = 0;         // Пин определяющий импульс
  int8_t micric_pin = 0;       // Пин определяющий концевик
  int32_t globalTransform = 0; // Угол котрый надо прибавить чтобы локальный угол превратить в глобальный
};
motorStruct motor[4]; // Все локальные данные по моторам

struct motorStructSend // Структура которая передается на верхний уровень
{
  int32_t status = 0;    // Передаются импульсы на мотор или нет в данный момент, вращается или нет
  float position = 0;    // Текущая позиция в градусах
  float destination = 0; // Цель назначение в позиции в градусах
};

//*********************************************************************
// Структура получаемых данных от Data к контроллеру Modul
struct Struct_Data2Modul
{
  uint32_t id = 0;      // Номер команды по порядку
  uint32_t command = 0; // Текущая команда к выполенению 0 - режим колибровки концевиков 1 - обычное управление моторами по углу
  float angle[4];       // Углы в которые нужно повернультя в глобальной системе

  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Data2Modul Data2Modul_receive;                          // Экземпляр структуры получаемых данных
const int size_structura_receive = sizeof(Data2Modul_receive); // Размер структуры с данными которые получаем

//*********************************************************************
// Структура в которой все главные переменные передаюся на высокий уровень от Modul к Data
struct Struct_Modul2Data
{
  uint32_t id = 0; // id команды

  uint32_t pinMotorEn = 0;  // Стутус пина управления драйвером моторов, включен драйвер или нет
  motorStructSend motor[4]; // Структура по состоянию моторов
  lidarStructSend lidar[4]; // Структура по состоянию лидаров
  uint32_t micric[4];       // Структура по состоянию концевиков
  spiStruct spi;            //Структура по состоянию обмена по шине SPI

  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Modul2Data Modul2Data_send;                                                                                             // Тут все переменные его характеризующие на низком уровне
const int size_structura_send = sizeof(Modul2Data_send);                                                                       // Размер структуры с данными которые передаем
const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше

#endif