#ifndef UZI_H
#define UZI_H

#define TrigPin 12
#define EchoPin 35
#define TIME_WAIT_UZI 40000 // Время которое ждем данные, они могут не придти и потеряться 50 милисекунд

volatile unsigned long time_start_uzi = 0; // Время начла измерений
volatile unsigned long time_end_uzi = 0;   // Время завершения измерений

volatile int Flag_uzi_good_data = 0; // Флаг что данные получены и можно обрабатывать
volatile bool Flag_uzi_wait = false; // Флаг что отправили импульс и ждем отклика или уже не ждем и можно слать новые
volatile uint8_t status_uzi = 0;     // Фаза прерывания или только начали замер или закончили
float distance_uzi = 0;              // Глобальная переменная в которую мы запоминаяем результаты измерения

bool flag_rcwl1601 = false; // Флаг для запуска измерений ультразвуковым датчиком

// Функция генерации импульса для ультразвукового датчика
void Triger_start()
{
    // Serial.println(" Triger_start... ");
    digitalWrite(TrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);
    status_uzi = 1; // Начали мпульс
}
// Функция которая вызывается прерыванием и в которой мы запоминаем время и взводим флаг для расчета в основном цикле
void interuptUzi()
{
    if (status_uzi == 1) // Если начало измерений
    {
        // Serial.println(" status_uzi == 1 ");
        time_start_uzi = micros(); // Запоминаем время
        Flag_uzi_wait = true;      // Флаг что ждем данные
        status_uzi = 2;
        return;
    }
    if (status_uzi == 2) // Если конец измерений
    {
        time_end_uzi = micros(); // Запоминаем время
        Flag_uzi_good_data = 1;  // Взводим флаг что данные пришли и можно обсчитать данные
    }
}

// Инициализация пинов и прерывания
void init_Uzi()
{
    Serial.println("======================================= init_Uzi ===================================== ");
    pinMode(TrigPin, OUTPUT);
    digitalWrite(TrigPin, LOW);

    pinMode(EchoPin, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(EchoPin), interuptUzi, CHANGE); // Прерывание
    // LOW вызывает прерывание, когда на порту LOW
    // CHANGE прерывание вызывается при смене значения на порту, с LOW на HIGH и наоборот
    // RISING прерывание вызывается только при смене значения на порту с LOW на HIGH
    // FALLING прерывание вызывается только при смене значения на порту с HIGH на LOW
    Serial.println("End init_Uzi. ");
}

// Функция фильтрующая(сглаживающая) значения берет старое с меньшим весом и новое с большим
float filtr_My(float old_, float new_, float ves_new_)
{
    return (old_ * (1.0 - ves_new_)) + (new_ * ves_new_);
}

void loopUzi()
{

    // Обработка импульса на ультразвуковой датчик
    if (Flag_uzi_good_data == 1) // Если пришел хороший ответ на прошлый импульс то считаем расстояние
    {
        Flag_uzi_good_data = 0;
        Flag_uzi_wait = false; // Данные не ждем больше и можно слать новый запрос

        float distance_uzi_temp = ((time_end_uzi - time_start_uzi) / 58.0) / 100.0; // Переводи в метры
        if (distance_uzi_temp > 2)                                                  // Ограничиваем измерние 2 метрами
            distance_uzi_temp = 2;
        // distance_uzi = (distance_uzi_temp * 0.90) + (distance_uzi * 0.10); // Небольшое сглаживание показаний
        distance_uzi = filtr_My(distance_uzi, distance_uzi_temp, 0.9);
        uzi1.distance = distance_uzi;
        //Serial.printf(" UZI Good ! distance = ", distance_uzi);
    }
    // Отмена ожидания данных от датчика по таймауту
    if (Flag_uzi_wait) // если ждем данные
    {
        if ((micros() - time_start_uzi) >= TIME_WAIT_UZI) // Если с момента генерации импульса прошло больше времени чем надо
        {
            Flag_uzi_good_data = -1; // Флаг что данных не дождались
            time_end_uzi = micros(); // Запоминаем время
        }
    }

    if (Flag_uzi_good_data == -1) // Если мы не дождались ответного сигнала
    {
        Flag_uzi_good_data = 0;
        Flag_uzi_wait = false; // Данные не ждем больше и можно слать новый запрос
        Serial.println("-02/");
        // Serial.println((time_end_uzi - time_start_uzi));
    }
}

// void inTimerUzi()
// {
//     // Отмена ожидания данных от датчика по таймауту
//     if (Flag_uzi_wait) // если ждем данные
//     {
//         if ((micros() - time_start_uzi) >= TIME_WAIT_UZI) // Если с момента генерации импульса прошло больше времени чем надо
//         {
//             Flag_uzi_good_data = -1; // Флаг что данных не дождались
//             time_end_uzi = micros(); //Запоминаем время
//         }
//     }
// }

#endif