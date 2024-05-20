#ifndef LASER80_H
#define LASER80_H

class CLaser80M
{
private:
    /* data */
    void clearBuf(); // Очистка буфера приема
public:
    CLaser80M(/* args */);
    ~CLaser80M();
    //*********************** ОБЬЯВЛЕНИЕ ФУНКЦИЙ *****************************************
    // Функции применимык к конкретному датчику, так как задается адрес датчика
    byte calcCs(byte *data_, byte len_);         // Расчет контрольной суммы. Берется массив всех оправляемых данных без последнего байта и суммируется побайтно, потом в бинарном виде инвертируются 1 в нолик и нолик в единицу и потом прибавляется 1
    void singleMeasurement(byte addr_);          // Одиночное измерение примерно 800 милисекунд
    void continuousMeasurement(byte addr_);      // Непрерывное измерение
    void stopMeasurement(byte addr_);            // Прекратить измерение
    bool setAddress(byte addr_);                 // Установка нового адрес на датчике
    bool readCache(byte addr_);                  // Считывание данных из буфера датчика результат измерения
    bool controlLaser(byte status_, byte addr_); // Управление лазером 1- Включен 0-Выключен

    // Функции применяются ко всем датчикам на линии и поэтому калибровку и прочее делать с одиночным датчиком
    void broadcastMeasurement(); // Единое измерние. Команда всем подключенным датчикам произвести измерение.Потом его надо считать с каждого датчика

    bool setDistanceModification(int8_t data_); // Модификация дистанции. Думаю что колибровка измерений. Можно в плюс или в минус
    bool setTimeInterval(byte data_);           // Установка инрервала вывода значения при настройке. Не понятно что это.
    bool setStartingPoint(byte data_);          // Устанвка точки откоторой считем расстояние. 1- от носа 0 - от зада
    bool setRange(byte range_);                 // Установление максимального диапзона измерений. Возможно 5,10,30,50,80 метров
    bool setFrequency(byte freq_);              // Установка частоты измерений, задается в герцах 3,5,10,20 только такие частоты
    bool setResolution(byte reso_);             // Установка разрешения измерения есди 1- то 1 мм, если 2 то 0,1 мм. Непонятно работает ли нет фактически и на чем сказывается (время измерения?)
    bool setTestPowerOn(byte data_);            // Установка нужно ли проводить тест датчика после включения питания. Значение 0 или 1
};

CLaser80M::CLaser80M(/* args */)
{
}

CLaser80M::~CLaser80M()
{
}

void CLaser80M::clearBuf()
{
    while (Serial2.available()) // Очищаем буфер
        Serial2.read();
}
//************************************************************************************
byte CLaser80M::calcCs(byte *data_, byte len_) // Расчет контрольной суммы. Берется массив всех оправляемых данных без последнего байта и суммируется побайтно, потом в бинарном виде инвертируются 1 в нолик и нолик в единицу и потом прибавляется 1
{
    byte ret;
    uint32_t sum = 0;
    // Serial.println(len_, DEC);
    for (int i = 0; i < (len_ - 1); i++) // Считаем без последнего байта
    {
        sum += data_[i];
        // Serial.print(data_[i], HEX);
        //  Serial.print("-");
    }
    // Serial.println(" |");
    // Serial.print(sum, DEC);
    // Serial.print(" ");
    // Serial.print(sum, HEX);
    // Serial.print(" ");
    // Serial.println(sum, BIN);
    ret = (sum ^ 0b1111111111111111) + 1; // инвертируем биты
    // Serial.print(ret, DEC);
    // Serial.print(" ");
    // Serial.print(ret, HEX);
    // Serial.print(" ");
    // Serial.println(ret, BIN);
    return ret;
}
// Одиночное измерение примерно 800 милисекунд
void CLaser80M::singleMeasurement(byte addr_)
{
    digitalWrite(PIN_LED, 1);
    clearBuf();
    byte buf[4] = {addr_, 0x06, 0x02, 0x00}; // Команда без последнего байта, там будет контрольная сумма, а пока 0х00
    buf[3] = calcCs(buf, 4);
    Serial2.write(buf, 4);
    delay(850);
    byte len = 11;
    byte bufRead[len];
    for (int i = 0; i < len; i++)
    {
        bufRead[i] = Serial2.read();
        printf("%X-", bufRead[i]);
    }
    digitalWrite(PIN_LED, 0);
    printf("\n");
    byte sot = (bufRead[3] - 0x30) * 100;       // По таблице ASCII отнимаем 48 и получаем сколько сотен метров
    byte des = (bufRead[4] - 0x30) * 10;        // По таблице ASCII отнимаем 48 и получаем сколько десятков метров
    byte met = (bufRead[5] - 0x30) * 1;         // По таблице ASCII отнимаем 48 и получаем сколько единиц метров
    float desMet = (bufRead[7] - 0x30) * 0.1;   // По таблице ASCII отнимаем 48 и получаем сколько десятых долей метра
    float sotMet = (bufRead[8] - 0x30) * 0.01;  // По таблице ASCII отнимаем 48 и получаем сколько сотых долей метра
    float tysMet = (bufRead[9] - 0x30) * 0.001; // По таблице ASCII отнимаем 48 и получаем сколько тысячных долей метра
    float distance = sot + des + met + desMet + sotMet + tysMet;

    printf("Meas= %i - %i - %i . %.1f %.2f %.3f | ", sot, des, met, desMet, sotMet, tysMet);
    printf("Distance= %f \n", distance);
    // if (bufRead[0] == addr_ && buf[1] == 0x06 && buf[2] == 0x82 && buf[3] == 0x45 && buf[4] == 0x52 && buf[4] == 0x52) // Проверка на ошибку. Возвращает ADDR 06 82"'E' 'R' 'R' '-' '-' '3X' '3X' ”CS
    // {
    //     printf("readMeasurement ERROR \n");
    //     return false;
    // }
}

// Непрерывное измерение
void CLaser80M::continuousMeasurement(byte addr_)
{
    byte buf[4] = {addr_, 0x06, 0x03, 0x00};
    buf[3] = calcCs(buf, 4);
    Serial2.write(buf, 4);
}
// Прекратить измерение
void CLaser80M::stopMeasurement(byte addr_)
{
    clearBuf();
    byte buf[4] = {addr_, 0x04, 0x02, 0x00};
    buf[3] = calcCs(buf, 4);
    Serial2.write(buf, 4);
    delay(10);
    buf[0] = Serial2.read();
    buf[1] = Serial2.read();
    buf[2] = Serial2.read();
    buf[3] = Serial2.read();

    if (buf[0] == addr_ && buf[1] == 0x04 && buf[2] == 0x82 && buf[3] == 0xFA)
    {
        printf("stopMeasurement ok \n");
    }
    else
    {
        printf("stopMeasurement ERROR \n");
    }
}

// Единое измерние. Команда всем подключенным датчикам произвести измерение.Потом его надо считать с каждого датчика
void CLaser80M::broadcastMeasurement()
{
    byte buf[4] = {0xFA, 0x06, 0x06, 0xFA};
    Serial2.write(buf, 4);
}
// Считывание данных из буфера датчика результат измерения
bool CLaser80M::readCache(byte addr_)
{
    clearBuf();
    byte buf[4] = {addr_, 0x06, 0x07, 0x00};
    buf[3] = calcCs(buf, 4); // Считаем контрольную сумму и записываем последним байтом
    Serial2.write(buf, 4);
    delay(20);
    byte len = 11;
    byte bufRead[len];
    for (int i = 0; i < len; i++)
    {
        bufRead[i] = Serial2.read();
        printf("%X-", bufRead[i]);
    }

    if (bufRead[0] == addr_ && buf[1] == 0x06 && buf[2] == 0x82 && buf[3] == 0x45 && buf[4] == 0x52 && buf[4] == 0x52) // Проверка на ошибку. Возвращает ADDR 06 82"'E' 'R' 'R' '-' '-' '3X' '3X' ”CS
    {
        printf("readMeasurement ERROR \n");
        return false;
    }
    // String rez = bufRead[3] + bufRead[4] + bufRead[5] + "." + bufRead[6] + bufRead[6] + bufRead[8];

    // printf(" rez = %s \n", rez.c_str());
    // Serial.println("rezultat=" + rez);

    return true;
}
// Установка инрервала вывода значения при настройке. Не понятно что это.
bool CLaser80M::setTimeInterval(byte data_)
{
    clearBuf();
    if (data_ == 0) // через 0 секунду
    {
        byte buf[5] = {0xFA, 0x04, 0x05, 0x00, 0xFD};
        Serial2.write(buf, 5);
    }
    if (data_ == 1) // через 1 секунду
    {
        byte buf[5] = {0xFA, 0x04, 0x05, 0x01, 0xFC};
        Serial2.write(buf, 5);
    }
    delay(1000);
    byte bufRead[5];
    bufRead[0] = Serial2.read();
    bufRead[1] = Serial2.read();
    bufRead[2] = Serial2.read();
    bufRead[3] = Serial2.read();
    bufRead[4] = Serial2.read();
    printf("setTimeInterval DATA => %X %X %X %X %X\n", bufRead[0], bufRead[1], bufRead[2], bufRead[3], bufRead[4]);
    if (bufRead[0] == 0xFA && bufRead[1] == 0x04 && bufRead[2] == 0x85 && bufRead[3] == 0x7D)
    {
        printf("setTimeInterval ok \n");
        return true;
    }
    else
    {
        printf("setTimeInterval ERROR\n");
        return false;
    }
}
// Установка нового адрес на датчике Широковещательный команда, смотреть что датчик один в этот момент на шине.
bool CLaser80M::setAddress(byte addr_)
{
    clearBuf();
    byte buf[5] = {0xFA, 0x04, 0x01, addr_, 0x00};
    buf[4] = calcCs(buf, 5);
    Serial2.write(buf, 5);
    delay(1000);
    byte bufRead[5];
    bufRead[0] = Serial2.read();
    bufRead[1] = Serial2.read();
    bufRead[2] = Serial2.read();
    bufRead[3] = Serial2.read();
    bufRead[4] = Serial2.read();
    printf("setAddress DATA => %X %X %X %X %X\n", bufRead[0], bufRead[1], bufRead[2], bufRead[3], bufRead[4]);
    if (bufRead[0] == 0xFA && bufRead[1] == 0x04 && bufRead[2] == 0x81 && bufRead[3] == 0x81)
    {
        printf("setAddress ok \n");
        return true;
    }
    else
    {
        printf("setAddress ERROR\n");
        return false;
    }
}
// Модификация дистанции. Думаю что колибровка измерений. Можно в плюс или в минус
bool CLaser80M::setDistanceModification(int8_t data_)
{
    clearBuf();
    byte sign;
    if (data_ > 0) // Если корректировка плюсовая
    {
        sign = 0x2b;
    }
    else // если минусовая
    {
        sign = 0x2d;
    }
    byte data = abs(data_);
    byte buf[6] = {0xFA, 0x04, 0x06, sign, data, 0x00};
    // byte *bufAddr = (byte *)buf; // Создаем переменную в которую пишем адрес буфера в нужном формате
    buf[5] = calcCs(buf, 6);
    Serial2.write(buf, 6);
    delay(1000);
    byte bufRead[5];
    bufRead[0] = Serial2.read();
    bufRead[1] = Serial2.read();
    bufRead[2] = Serial2.read();
    bufRead[3] = Serial2.read();
    bufRead[4] = Serial2.read();
    printf("setAddress DATA => %X %X %X %X %X\n", bufRead[0], bufRead[1], bufRead[2], bufRead[3], bufRead[4]);
    if (bufRead[0] == 0xFA && bufRead[1] == 0x04 && bufRead[2] == 0x8B && bufRead[3] == 0x77)
    {
        printf("setDistanceModification ok \n");
        return true;
    }
    else
    {
        printf("setDistanceModification ERROR \n");
        return false;
    }
}
// Устанвка точки откоторой считем расстояние. 1- от носа 0 - от зада
bool CLaser80M::setStartingPoint(byte data_)
{
    clearBuf();
    if (data_ == 0) //
    {
        byte buf[5] = {0xFA, 0x04, 0x08, 0x00, 0xFA};
        Serial2.write(buf, 5);
    }
    if (data_ == 1) //
    {
        byte buf[5] = {0xFA, 0x04, 0x08, 0x01, 0xF9};
        Serial2.write(buf, 5);
    }
    delay(1000);
    byte bufRead[5];
    bufRead[0] = Serial2.read();
    bufRead[1] = Serial2.read();
    bufRead[2] = Serial2.read();
    bufRead[3] = Serial2.read();
    bufRead[4] = Serial2.read();
    printf("setAddress DATA => %X %X %X %X %X\n", bufRead[0], bufRead[1], bufRead[2], bufRead[3], bufRead[4]);

    if (bufRead[0] == 0xFA && bufRead[1] == 0x04 && bufRead[2] == 0x88 && bufRead[3] == 0x7A)
    {
        printf("setStartingPoint ok \n");
        return true;
    }
    else
    {
        printf("setStartingPoint ERROR \n");
        return false;
    }
}

// Установка нужно ли проводить тест датчика после включения питания. Значение 0 или 1
bool CLaser80M::setTestPowerOn(byte data_)
{
    clearBuf();
    if (data_ == 0) //
    {
        byte buf[5] = {0xFA, 0x04, 0x0D, 0x00, 0xF5}; // Нет теста
        Serial2.write(buf, 5);
    }
    if (data_ == 1) //
    {
        byte buf[5] = {0xFA, 0x04, 0x0D, 0x01, 0xF4}; // Тестировать
        Serial2.write(buf, 5);
    }
    return true;
}
// Установление максимального диапзона измерений. Возможно 5,10,30,50,80 метров
bool CLaser80M::setRange(byte range_)
{
    clearBuf();
    if (range_ == 5) //
    {
        byte buf[5] = {0xFA, 0x04, 0x09, 0x05, 0xF4};
        Serial2.write(buf, 5);
    }
    if (range_ == 10) //
    {
        byte buf[5] = {0xFA, 0x04, 0x09, 0x0A, 0xEE};
        Serial2.write(buf, 5);
    }
    if (range_ == 30) //
    {
        byte buf[5] = {0xFA, 0x04, 0x09, 0x1E, 0xDB};
        Serial2.write(buf, 5);
    }
    if (range_ == 50) //
    {
        byte buf[5] = {0xFA, 0x04, 0x09, 0x32, 0xC7};
        Serial2.write(buf, 5);
    }
    if (range_ == 80) //
    {
        byte buf[5] = {0xFA, 0x04, 0x09, 0x50, 0xA9};
        Serial2.write(buf, 5);
    }
    delay(1000);
    byte bufRead[5];
    bufRead[0] = Serial2.read();
    bufRead[1] = Serial2.read();
    bufRead[2] = Serial2.read();
    bufRead[3] = Serial2.read();
    bufRead[4] = Serial2.read();
    printf("setRange DATA => %X %X %X %X %X\n", bufRead[0], bufRead[1], bufRead[2], bufRead[3], bufRead[4]);
    if (bufRead[0] == 0xFA && bufRead[1] == 0x04)
    {
        printf("setRange ok \n");
        return true;
    }
    else
    {
        printf("setRange ERROR\n");
        return false;
    }
}

// Установка разрешения измерения есди 1- то 1 мм, если 2 то 0,1 мм. Непонятно работает ли нет фактически и на чем сказывается (время измерения?)
bool CLaser80M::setResolution(byte reso_)
{
    clearBuf();
    if (reso_ == 1) //
    {
        byte buf[5] = {0xFA, 0x04, 0x0C, 0x01, 0xF5};
        Serial2.write(buf, 5);
    }
    if (reso_ == 2) //
    {
        byte buf[5] = {0xFA, 0x04, 0x0C, 0x02, 0xF4};
        Serial2.write(buf, 5);
    }
    delay(1000);
    byte bufRead[5];
    bufRead[0] = Serial2.read();
    bufRead[1] = Serial2.read();
    bufRead[2] = Serial2.read();
    bufRead[3] = Serial2.read();
    bufRead[4] = Serial2.read();
    printf("setResolution DATA => %X %X %X %X %X\n", bufRead[0], bufRead[1], bufRead[2], bufRead[3], bufRead[4]);
    if (bufRead[0] == 0xFA && bufRead[1] == 0x04)
    {
        printf("setResolution ok \n");
        return true;
    }
    else
    {
        printf("setResolution ERROR\n");
        return false;
    }
}
// Установка частоты измерений, задается в герцах 3,5,10,20 только такие частоты
bool CLaser80M::setFrequency(byte freq_)
{
    clearBuf();
    if (freq_ == 3) //  примерно 3 Hz
    {
        byte buf[5] = {0xFA, 0x04, 0x0A, 0x00, 0xF8};
        Serial2.write(buf, 5);
    }
    if (freq_ == 5) //  фактически 7 Hz
    {
        byte buf[5] = {0xFA, 0x04, 0x0A, 0x05, 0xF3};
        Serial2.write(buf, 5);
    }
    if (freq_ == 10) //  фактически 8 Hz
    {
        byte buf[5] = {0xFA, 0x04, 0x0A, 0x0A, 0xEE};
        Serial2.write(buf, 5);
    }
    if (freq_ == 20) //  фактически 18 Hz
    {
        byte buf[5] = {0xFA, 0x04, 0x0A, 0x14, 0xE4};
        Serial2.write(buf, 5);
    }
    delay(1000);
    byte bufRead[5];
    bufRead[0] = Serial2.read();
    bufRead[1] = Serial2.read();
    bufRead[2] = Serial2.read();
    bufRead[3] = Serial2.read();
    bufRead[4] = Serial2.read();
    printf("setFrequency DATA => %X %X %X %X %X\n", bufRead[0], bufRead[1], bufRead[2], bufRead[3], bufRead[4]);
    if (bufRead[0] == 0xFA && bufRead[1] == 0x04)
    {
        printf("setFrequency ok \n");
        return true;
    }
    else
    {
        printf("setFrequency ERROR\n");
        return false;
    }
}
// Управление лазером 1- Включен 0-Выключен
bool CLaser80M::controlLaser(byte status_, byte addr_)
{
    clearBuf();
    if (status_ == 0) //  Выключить
    {
        byte buf[5] = {addr_, 0x06, 0x05, 0x00, 0x00};
        buf[4] = calcCs(buf, 5);
        Serial2.write(buf, 5);
    }
    if (status_ == 1) //  Включить
    {
        byte buf[5] = {addr_, 0x06, 0x05, 0x01, 0x00};
        buf[4] = calcCs(buf, 5);
        Serial2.write(buf, 5);
    }
    delay(1000);
    byte bufRead[5];
    bufRead[0] = Serial2.read();
    bufRead[1] = Serial2.read();
    bufRead[2] = Serial2.read();
    bufRead[3] = Serial2.read();
    bufRead[4] = Serial2.read();
    printf("controlLaser DATA => %X %X %X %X %X\n", bufRead[0], bufRead[1], bufRead[2], bufRead[3], bufRead[4]);
    if (bufRead[0] == addr_ && bufRead[1] == 0x06 && bufRead[2] == 0x85 && bufRead[3] == 0x01)
    {
        printf("controlLaser ok \n");
        return true;
    }
    else
    {
        printf("controlLaser ERROR \n");
        return false;
    }
}
#endif