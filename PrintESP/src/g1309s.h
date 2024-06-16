#ifndef G1309S_H
#define G1309S_H

/*
    Класс для кодированиепечати для картриджа G1309S
*/
class CG1309s
{
private:
    /* data */
public:
    CG1309s(/* args */);
    ~CG1309s();
    void printLine(uint8_t *line_, uint8_t size_);         // Печать массива с данными
    //void line10to150(uint8_t *line15_, uint8_t *line150_); // Метод преобразования из массива 10 элементов в 150 элементов

    //void line150toCode24(uint8_t *line150_, uint16_t *code24_); // Метод преобразования из массива 150 элементов в 11 кодовых значений в 2 байта

    void line15toCode24(uint8_t *line15_, uint16_t *code24_); // Метод преобразования из массива 15 элементов в 11 кодовых значений в 2 байта
    void code24toCode48(uint16_t *code24_, uint8_t *code48_);   // Метод преобразования из массива 24 элементов в массив 48 элементов по 1 байту для передачи по SPI в правильном порядке
    void code48toCode48Arr(uint8_t *code48_, uint8_t *code48Arr_);   // Метод записи массива одиночного в многомерный
};

CG1309s::CG1309s(/* args */)
{
}

CG1309s::~CG1309s()
{
}

// // Метод преобразования из массива 10 элементов в 150 элементов
// void CG1309s::line10to150(uint8_t *line15_, uint8_t *line150_)
// {
//     int count = 0;
//     for (int i = 0; i < 150; i++)
//     {
//         count = i / 15; // Тут только целое число, дробный остаток отбратывается
//         line150_[i] = line15_[count];
//     }
// }
// Печать массива с данными
void CG1309s::printLine(uint8_t *line_, uint8_t size_)
{
    printf("printLine -> count \n", size_);
    for (int i = 0; i < size_; i++)
    {
        printf("%i ", line_[i]);
    }
    printf("\n");
}
// Метод преобразования из массива 15 элементов в 11 кодовых значений в 2 байта
void CG1309s::line15toCode24(uint8_t *line15_, uint16_t *code24_)
{
    int count = 0;
    uint8_t line150[150];
    for (int i = 0; i < 150; i++)
    {
        count = i / 10; // Тут только целое число, дробный остаток отбратывается
        line150[i] = line15_[count];
    }

    for (int i = 0; i < 24; i++) // Очищаем массив нулями
    {
        code24_[i] = 0;
    }

    u_int8_t level = 0;    // Уровень на котором разбираем пикселы. Уровень каждый кратен 11 пикселам
    u_int16_t bitNum = 0;  // Номер бита который меняем. Завитит от уровня на каком смотрим пикселы.
    u_int8_t numPixel = 0; // Номер пиксела на уровне.
    u_int8_t gate = 0;     // Номер элемента в массиве который меняем.
    for (int i = 0; i < 150; i++)
    {
        level = i / 11; // Целое число. показывает на каком мы уровне разбираем пикселы
        switch (level)  // В зависимости от уровня меняем разные биты в числе. Тут указываем число с которым потом складываем. и какой бит меняем. Отсчет справа.
        {
        case 0:
            bitNum = 0x0800; // 12 бит меняется на 1;
            break;
        case 1:
            bitNum = 0x0400; // 11;
            break;
        case 2:
            bitNum = 0x0200; // 10;
            break;
        case 3:
            bitNum = 0x0100; // 9;
            break;
        case 4:
            bitNum = 0x4000; // 15;
            break;
        case 5:
            bitNum = 0x2000; // 14;
            break;
        case 6:
            bitNum = 0x1000; // 13;
            break;
        case 7:
            bitNum = 0x0008; // 4;
            break;
        case 8:
            bitNum = 0x0010; // 5;
            break;
        case 9:
            bitNum = 0x0020; // 6;
            break;
        case 10:
            bitNum = 0x0040; // 7;
            break;
        case 11:
            bitNum = 0x0001; // 1;
            break;
        case 12:
            bitNum = 0x0002; // 2;
            break;
        case 13:
            bitNum = 0x0004; // 3;
            break;
        }

        numPixel = i - (level * 11); // Целое число. показывает какой пиксел на уровне мы обрабатываем
        switch (numPixel)
        {
        case 0:
            gate = 7;
            break;
        case 1:
            gate = 14;
            break;
        case 2:
            gate = 6;
            break;
        case 3:
            gate = 5;
            break;
        case 4:
            gate = 12;
            break;
        case 5:
            gate = 4;
            break;
        case 6:
            gate = 3;
            break;
        case 7:
            gate = 10;
            break;
        case 8:
            gate = 2;
            break;
        case 9:
            gate = 1;
            break;
        case 10:
            gate = 8;
            break;
        }
        if (line150[i] == 1) // Если есть пиксель то меняем нужный бит в кодовом значении
        {
            // printf("level %i numPixel %i bitNum %i gate %i \n", level, numPixel, bitNum, gate);
            //  uint16_t data = 0;
            //  Serial.print("BIN1= ");
            //  Serial.print(code24_[gate], BIN);
            //  Serial.print(" HEX= ");
            //  Serial.println(code24_[gate], HEX);

            code24_[gate] = code24_[gate] | bitNum; // поразрядная дизъюнкция (операция ИЛИ или поразрядное сложение). Возвращает 1, если хотя бы один из соответствующих разрядов обоих чисел равен 1

            // Serial.print("BIN2= ");
            // Serial.print(code24_[gate], BIN);
            // Serial.print(" HEX= ");
            // Serial.println(code24_[gate], HEX);
        }
    }

    code24_[22] = code24_[7]; // Дублируем в нужные ячейки. Наверное можно если не дублировать а дополнительно посчитать пикселы получить лучше разрешение...?!
    code24_[21] = code24_[14];
    code24_[13] = code24_[6];
    code24_[20] = code24_[5];
    code24_[19] = code24_[12];
    code24_[11] = code24_[4];
    code24_[18] = code24_[3];
    code24_[17] = code24_[10];
    code24_[9] = code24_[2];
    code24_[16] = code24_[1];
    code24_[15] = code24_[8];

    code24_[0] = 0x8080;  // Заголовок
    code24_[23] = 0x0320; // конец

    // for (int i = 0; i < 24; i++)
    // {
    //     Serial.print(" i= ");
    //     Serial.print(i);
    //     Serial.print(" = ");
    //     Serial.println(code24_[i], HEX);
    // }
}
// Метод преобразования из массива 24 элементов в массив 48 элементов по 1 байту для передачи по SPI в правильном порядке
void CG1309s::code24toCode48(uint16_t *code24_, uint8_t *code48_)
{
    uint8_t *temp48 = (uint8_t *)code24_; // Меняем размер адреса на однобайтный
    for (int i = 0; i < 48; i = i + 2)
    {
        code48_[i] = temp48[i + 1];
        code48_[i + 1] = temp48[i];
    }
}

// Метод записи массива одиночного в многомерный
void CG1309s::code48toCode48Arr(uint8_t *code48_, uint8_t *code48Arr_)

{
    for (int i = 0; i < 48; i++)
    {
        code48Arr_[i] = code48_[i];
    }
} 























#endif