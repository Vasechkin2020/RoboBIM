
#ifndef LED43_H
#define LED43_H

#include <FastLED.h>

#define DATA_PIN 15 // Номер пина на который подключены

enum color
{
    black = 0,
    red = 1,
    green = 2,
    blue = 3,
    white = 4,
    yellow = 5
};

class led43
{
private:
    uint32_t a = 0;
    uint32_t b = 0;

    bool led_flag_run = 0;      // Флаг на светодиоды мигающий алгоритм
    uint8_t led_count_run = 0;  // Счетчик на светодиоды бегущий алгоритм
    bool led_flag_runL = 0;     // Флаг на светодиоды мигающий алгоритм
    uint8_t led_count_runL = 0; // Счетчик на светодиоды бегущий алгоритм

    bool led_flag_blink = 0;      // Флаг на светодиоды мигающий алгоритм
    uint8_t led_count_blink = 0;  // Счетчик на светодиоды мигающий алгоритм
    bool led_flag_blink2 = 0;     // Флаг на светодиоды мигающий алгоритм
    uint8_t led_count_blink2 = 0; // Счетчик на светодиоды мигающий алгоритм

    CRGB leds[NUM_LEDS]; // Define the array of leds
    // определяем перечисление

public:
    led43(/* args */);
    ~led43();
    void _init();
    void _setBrightness(byte level_);                             // становка яркости свечения
    void _show();                                                 // Перепрошивка светодиодов
    void _ledUp(byte a_, byte b_, color color_);                  // Включить или выключить группу светодиодов центральную на 7 штук
    void _translate(SLed led_);                                   // Запись пришедших данных в массив для отображения до 36 светодиода. Серединака светодиодов управляется с драйвера платы
    void _ledTranslate(byte i_, uint8_t color_);                  // Назначение светодиодов
    void _test();                                                 // Тестирование светодиодов

    // void run0();                                                 // Основной цикл мигания светодиодами
    // void run1();                                                 // Основной цикл мигания светодиодами
    // void run2();                                                 // Основной цикл мигания светодиодами
    // void ledDown(byte a_, byte b_);                              // выключить группу светодиодов центральную на 7 штук
    // void ledBlink(byte delay_, byte a_, byte b_, color color_);  // Мигать нужными светодиодами с нужной задержкой (частотой)
    // void ledBlink2(byte delay_, byte a_, byte b_, color color_); // Мигать нужными светодиодами с нужной задержкой (частотой)
    // void ledRunR(byte delay_, byte a_, byte b_, color color_);   // Бегать по нужным светодиодам с нужной частотой и яркостью и направлением
    // void ledRunL(byte delay_, int a_, byte b_, color color_);    // Бегать по нужным светодиодам с нужной частотой и яркостью и направлением
    //int flag_led_off = 0;                                        // Статаус что cветодиоды выключены
};

led43::led43(/* args */)
{
}
void led43::_init()
{
    Serial.println(String(millis()) + " Start led2812.init ...");
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); // GRB ordering is assumed
    _setBrightness(50);
    _ledUp(0, 42, white);
    _show();
    delay(500);
    _ledUp(0, 42, black);
    _show();
    Serial.println(String(millis()) + " End led2812.init ...");
}

led43::~led43()
{
}

// Установка яркости свечения
void led43::_setBrightness(byte level_)
{
    FastLED.setBrightness(level_);
}
// Перепрошивка светодиодов
void led43::_show()
{
    FastLED.show();
}
// Функция тестирования колец светодиодов
void led43::_test()
{
    Serial.println(String(micros()) + " Start led2812.test ...");
    _setBrightness(50);
    _ledUp(0, 12, green);
    _ledUp(13, 23, red);
    _show();
    delay(500);
    //-------------------------------------
    _ledUp(0, 42, black);
    _show();
    //-------------------------------------
    _ledUp(24, 35, red);
    _show();
    delay(500);
    //-------------------------------------
    _ledUp(0, 42, black);
    _show();
    //-------------------------------------
    _ledUp(36, 42, white);
    _show();
    delay(500);
    //-------------------------------------
    _ledUp(0, 42, black);
    _show();
    Serial.println(String(micros()) + " End led2812.test ...");
}
// Запись пришедших данных в массив для отображения до 36 светодиода. Серединака светодиодов управляется с драйвера платы
void led43::_translate(SLed led_)
{
    for (int i = 0; i < 36; i++)
    {
        _ledTranslate(i, led_.led[i]);
    }
}
// // Основной цикл мигания светодиодами
// void led43::run0()
// {
//     ledDown(0, 43);
//     show();
//     flag_led_off = 1; // светодиоды выключены
// }
// // Основной цикл мигания светодиодами
// void led43::run1()
// {
//     ledBlink(50, 24, 35, green);
//     ledBlink2(10, 36, 43, white);
//     ledRunR(2, 1, 13, red);
//     ledRunL(2, 12, 23, red);
//     setBrightness(20);
//     show();
//     flag_led_off = 0; // светодиоды включены
// }
// // Освещение всего
// void led43::run2()
// {
//     ledUp(0, 43, white);
//     setBrightness(255);
//     show();
//     flag_led_off = 0; // светодиоды включены
// }

// void led43::ledDown(byte a_, byte b_)
// {
//     for (byte i = a_; i <= b_; i++)
//     {
//         leds[i] = CRGB::Black;
//     }
// }
void led43::_ledUp(byte a_, byte b_, color color_)
{
    switch (color_)
    {
    case black:
        for (byte i = a_; i <= b_; i++)
        {
            leds[i] = CRGB::Black;
        }
        break;
    case red:
        for (byte i = a_; i <= b_; i++)
        {
            leds[i] = CRGB::Red;
        }
        break;
    case green:
        for (byte i = a_; i <= b_; i++)
        {
            leds[i] = CRGB::Green;
        }
        break;
    case blue:
        for (byte i = a_; i <= b_; i++)
        {
            leds[i] = CRGB::Blue;
        }
        break;
    case yellow:
        for (byte i = a_; i <= b_; i++)
        {
            leds[i] = CRGB::Yellow;
        }
        break;
    case white:
        for (byte i = a_; i <= b_; i++)
        {
            leds[i] = CRGB::White;
        }
        break;

    default:
        for (byte i = a_; i <= b_; i++)
        {
            leds[i] = CRGB::White;
        }
        break;
    }
}
// Преобразование из чисел в имена и запись в нужные ячейки
void led43::_ledTranslate(byte i_, uint8_t color_)
{
    switch (color_)
    {
    case 0:
        leds[i_] = CRGB::Black;
        break;
    case 1:
        leds[i_] = CRGB::Red;
        break;
    case 2:
        leds[i_] = CRGB::Green;
        break;
    case 3:
        leds[i_] = CRGB::Blue;
        break;
    case 4:
        leds[i_] = CRGB::White;
        break;
    case 5:
        leds[i_] = CRGB::Yellow;
        break;
    }
}
// void led43::ledBlink(byte delay_, byte a_, byte b_, color color_) // Мигать нужными светодиодами с нужной задержкой (частотой) и яркостью
// {
//     if (led_count_blink == delay_)
//     {
//         led_count_blink = 0;
//         if (led_flag_blink == 0)
//         {
//             _ledUp(a_, b_, color_);
//             led_flag_blink = 1;
//         }
//         else
//         {
//             _ledDown(a_, b_);
//             led_flag_blink = 0;
//         }
//     }
//     else
//     {
//         led_count_blink++;
//     }
// }
// void led43::ledBlink2(byte delay_, byte a_, byte b_, color color_) // Мигать нужными светодиодами с нужной задержкой (частотой) и яркостью
// {
//     if (led_count_blink2 == delay_)
//     {
//         led_count_blink2 = 0;
//         if (led_flag_blink2 == 0)
//         {
//             _ledUp(a_, b_, color_);
//             led_flag_blink2 = 1;
//         }
//         else
//         {
//             _ledDown(a_, b_);
//             led_flag_blink2 = 0;
//         }
//     }
//     else
//     {
//         led_count_blink2++;
//     }
// }
// void led43::ledRunR(byte delay_, byte a_, byte b_, color color_) // Бегать по нужными светодиодами с нужной задержкой (частотой) и яркостью
// {
//     static uint8_t led_num_run = a_; // Номер светодиода с которым работаем бегущий алгоритм
//     if (led_count_run == delay_)
//     {
//         led_count_run = 0;
//         if (led_flag_run == 0)
//         {
//             _ledUp(led_num_run, led_num_run, color_);
//             led_flag_run = 1;
//         }
//         else
//         {
//             _ledDown(led_num_run, led_num_run);
//             led_flag_run = 0;
//             led_num_run++;
//             if (led_num_run == b_) // Если дошли до последнего светодиода и его погасили то переходим на первый
//                 led_num_run = a_;
//         }
//     }
//     else
//     {
//         led_count_run++;
//     }
// }
// void led43::ledRunL(byte delay_, int a_, byte b_, color color_) // Бегать по нужными светодиодами с нужной задержкой (частотой) и яркостью
// {
//     static int led_num_run = b_; // Номер светодиода с которым работаем бегущий алгоритм
//     if (led_count_runL == delay_)
//     {
//         led_count_runL = 0;
//         if (led_flag_runL == 0)
//         {
//             _ledUp(led_num_run, led_num_run, color_);
//             led_flag_runL = 1;
//         }
//         else
//         {
//             _ledDown(led_num_run, led_num_run);
//             led_flag_runL = 0;
//             led_num_run--;
//             // Serial.println(led_num_run);
//             if (led_num_run == (a_ - 1)) // Если дошли до последнего светодиода и его погасили то переходим на первый
//                 led_num_run = (b_);
//         }
//     }
//     else
//     {
//         led_count_runL++;
//     }
// }

#endif