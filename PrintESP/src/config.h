#ifndef CONFIG_H
#define CONFIG_H

// Контакты для светодиодов:
const int PIN_LED_RED = 33; // Для красного светодиода перепаял
// Нельзя использовать нулевой пин Не работает пин. При инициализации SPI_slave функция его занимает и дает всегда половину напряжения и он не реагирует на команды.
const int PIN_LED_BLUE = 32;
const int PIN_ANALIZ = 4;

uint8_t line15_0[15]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Массив изображения в 1 линию из 15 точек сверху вниз. 0 сверху - 9 внизу
uint8_t line15_1[15]{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // Массив изображения в 1 линию из 15 точек сверху вниз. 0 сверху - 9 внизу

uint8_t line15_2[15]{1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1}; // Массив изображения в 1 линию из 15 точек сверху вниз. 0 сверху - 9 внизу
uint8_t line15_3[15]{0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0}; // Массив изображения в 1 линию из 15 точек сверху вниз. 0 сверху - 9 внизу
uint8_t line15_4[15]{1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1}; // Массив изображения в 1 линию из 15 точек сверху вниз. 0 сверху - 9 внизу
uint8_t line15_5[15]{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}; // Массив изображения в 1 линию из 15 точек сверху вниз. 0 сверху - 9 внизу

// Массив изображения в 1 линию из 150 точек сверху вниз. То что кодируем
uint16_t code24[24];                                                                                                                                                                                                     // Массив двухбайтных элементов в которых битами отмечены пикселы которые надо напечатать.
uint16_t code24null[24];                                                                                                                                                                                                 //{0x8080, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0320};                                                                                                               // Массив двухбайтных элементов в которых битами отмечены пикселы которые надо напечатать.
uint16_t code24free[24]{0x0020, 0x0320, 0x0020, 0x0320, 0x0020, 0x0320, 0x0020, 0x0320, 0x0020, 0x0320, 0x0020, 0x0320, 0x0020, 0x0320, 0x0020, 0x0320, 0x0020, 0x0320, 0x0020, 0x0320, 0x0020, 0x0320, 0x0020, 0x0320}; // Массив двухбайтных элементов в которых битами отмечены пикселы которые надо напечатать.

uint8_t code48[48];     // Массив однобайтный в формате отправки по SPI
uint8_t code48null[48]; // Массив однобайтный в формате отправки по SPI
uint8_t code48free[48]; // Массив однобайтный в формате отправки по SPI

float filtr_My(float old_, float new_, float ves_new_); // Просто предолбявление функции. Реализация дальше. что-бы компилятор не ругался

volatile bool flag_timer_1sec = false;
volatile bool flag_timer_10millisec = false;
volatile bool flag_timer_50millisec = false;
volatile bool flag_timer_60sec = false;
volatile int count_timer_10millisec = 0; // Счетчик для запуска обработки движения моторов в лупе по флагу
volatile int count_timer_50millisec = 0; // Счетчик для запуска каждые 50 милисекунд
volatile int count_timer_1sec = 0;       // Счетчик для запуска
volatile int count_timer_60sec = 0;      // Счетчик для запуска

hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
hw_timer_t *timer3 = NULL;

//*********************************************************************
// Структура по обратной связи по обмену по шине SPI
struct SSpi
{
  uint32_t all = 0;
  uint32_t bed = 0;
};

// Структура в которой все собранные данные передаются из Print к Data
struct SPrint2Data
{
  uint32_t id = 0; // id команды
  SSpi spi;        // Структура по состоянию обмена по шине SPI

  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

SPrint2Data Print2Data_send;                             // Тело робота. тут все переменные его характеризующие на низком уровне
const int size_structura_send = sizeof(Print2Data_send); // Размер структуры с данными которые передаем

//*********************************************************************

struct SControlPrint
{
  uint32_t mode = 0; // Текущий режим работы 0 -
};

// Структура получаемых данных из Data к Print
struct SData2Print
{
  uint32_t id = 0; // Id команды
  SControlPrint modePrint; // Режим печати
  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

SData2Print Data2Print_receive;                                // Экземпляр структуры получаемых данных
const int size_structura_receive = sizeof(Data2Print_receive); // Размер структуры с данными которые получаем

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
#endif