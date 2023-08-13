#ifndef MY_REMOTEXY_H
#define MY_REMOTEXY_H

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// определение режима соединения и подключение библиотеки RemoteXY
#define REMOTEXY_MODE__ESP32CORE_WIFI_POINT
#include <RemoteXY.h>

// настройки соединения НЕ ИСПОЛЬЗУЮТСЯ ТАК КАК ОТКЛЮЧИЛ ВНУТРИ БИБЛИОТЕКИ СОЗДАНИЕ СЕТИ И СОЗДАЮ СЕТЬ САМ
#define REMOTEXY_WIFI_SSID "MyNet"
#define REMOTEXY_WIFI_PASSWORD "33445566"
#define REMOTEXY_SERVER_PORT 6300

// // конфигурация интерфейса
// #pragma pack(push, 1)
// uint8_t RemoteXY_CONF[] =
//   { 255,1,0,0,0,11,0,11,13,0,
//   4,0,54,24,7,18,2,26 };

// // структура определяет все переменные и события вашего интерфейса управления
// struct {

//     // input variables
//   int8_t slider_1; // =0..100 положение слайдера

//     // other variable
//   uint8_t connect_flag;  // =1 if wire connected, else =0

// } RemoteXY;
// #pragma pack(pop)

// Вариант с управлением только моторами движением по радиусу
// конфигурация интерфейса
// #pragma pack(push, 1)
// uint8_t RemoteXY_CONF[] =
//     {255, 2, 0, 0, 0, 32, 0, 10, 13, 0,
//      4, 176, 1, 15, 98, 15, 2, 26, 2, 1,
//      30, 46, 40, 16, 2, 26, 31, 31, 83, 116,
//      97, 114, 116, 0, 83, 116, 111, 112, 0};

// // структура определяет все переменные и события вашего интерфейса управления
// struct MyStruct
// {
//   // input variables
//   int8_t radius;     // =-100..100 положение слайдера
//   uint8_t startStop; // =1 если переключатель включен и =0 если отключен

//   // other variable
//   uint8_t connect_flag; // =1 if wire connected, else =0

// } RemoteXY;
// #pragma pack(pop)

// Вариант с управлением камерой и светодиодами и радиусом
#pragma pack(push, 1)
// uint8_t RemoteXY_CONF[] =
//   { 255,4,0,0,0,44,0,13,13,0,
//   2,0,32,49,22,11,2,26,31,31,
//   79,78,0,79,70,70,0,4,176,0,
//   31,88,9,2,26,4,0,90,2,9,
//   60,96,26,3,133,14,3,61,13,6,
//   26 };
//   #pragma pack(push, 1)

uint8_t RemoteXY_CONF[] = // 59 bytes
    {255, 5, 0, 0, 0, 52, 0, 16, 13, 0, 2, 0, 32, 49, 22, 11, 2, 26, 31, 31,
     79, 78, 0, 79, 70, 70, 0, 4, 176, 0, 31, 88, 9, 2, 26, 4, 0, 90, 2, 9,
     60, 96, 26, 3, 131, 10, 3, 33, 12, 6, 26, 3, 130, 61, 3, 22, 12, 134, 26};

// структура определяет все переменные и события вашего интерфейса управления
struct
{

  // input variables
  uint8_t startStop; // =1 если переключатель включен и =0 если отключен
  int8_t radius;     // =-100..100 положение слайдера
  int8_t camera;     // =0..100 положение слайдера
  uint8_t led;       // =0 если переключатель в положении A, =1 если в положении B, =2 если в положении C, ...
  uint8_t servo;     // =0 если переключатель в положении A, =1 если в положении B, =2 если в положении C, ...

  // other variable
  uint8_t connect_flag; // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

void changeDataFromRemoteXY()
{
    flag_newControlData = 1; // Устанавливаем флаг что есть новые данные по ручному управлению
    Data2Driver_receive.startStop = RemoteXY.startStop;
    Data2Driver_receive.speed = FIX_SPEED; // При ручном управлении скорость постоянная
    Data2Driver_receive.angle_camera = RemoteXY.camera;
    Data2Driver_receive.led = RemoteXY.led;
    Data2Driver_receive.servo = RemoteXY.servo;
    
    //Преобразование радиуса в нужный вид
    float radius = map(RemoteXY.radius, -100, 100, -MAX_RADIUS * 1000, MAX_RADIUS * 1000) / 1000.0; //Преобразуем диапазон из Remote XY от -100 до 100 в допустимый диапазон, только челые числа функция использует
    if (radius > 0 ) 
    {
       radius = MAX_RADIUS - radius + 0.001; // Прибавляем чуть-чуть чтобы радиус не получался 0 на краях
    }
    if (radius < 0 ) 
    {
       radius = -MAX_RADIUS - radius - 0.001; // Отнимаем чуть-чуть чтобы радиус не получался 0 на краях
    }
    Data2Driver_receive.radius = radius; 
}

void printRemoteXY()
{

  printf(" connect_flag= %i ", RemoteXY.connect_flag);
  printf(" startStop= %i ", RemoteXY.startStop);
  printf(" radius= %i ", RemoteXY.radius);
  printf(" servo= %i ", RemoteXY.servo);
  printf(" led= %i ", RemoteXY.led);
  printf(" camera= %i \n", RemoteXY.camera);
  // printf(" --------------------------- \n");
}
#endif