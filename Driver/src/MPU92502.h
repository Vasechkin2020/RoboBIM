
#include <EEPROM.h>
#include <Kalman.h>

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

//#define MPU9255_CS   53  //cs		  Чип селект для SPI для 9255
#define MPU9255_Address   0x69  //cs		 Адрес для I2C

int   magD;                // Направление по магнетрометру
bool flag_DataReady_AccGyro = false;  // Флаг статуса готовности данных гироскопа-Аксельрометра
long now_time;			   //Переменная для запоминания текущего времени

uint8_t buffer_acc_gyro[14];			 // Буфер для считывания данных аксельрометра


int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t temp9255;
uint16_t acc_delitel, gyro_delitel;

float accX, accY, accZ;
float accXoffset, accYoffset, accZoffset;
float angleAccX, angleAccY;
float angleX, angleY, angleZ;
float Temperatura9255;
bool  StatusColibrAccGyro = false;
float gyroX, gyroY, gyroZ;
float gyroXoffset, gyroYoffset, gyroZoffset;
float angleGyroX, angleGyroY, angleGyroZ; 
float angleGyro_Delta_X, angleGyro_Delta_Y, angleGyro_Delta_Z;
float intervalAccGyro, pre_intervalAccGyro;	 // для расчета интервала который произошел между прерываниями

float angleCompX;		// Данные после комплементарного фильтра
float angleCompY;
float angleCompZ;

float angleCompX2;		// Данные после комплементарного фильтра
float angleCompY2;
float angleCompZ2;

float X_angle_comp; // Переменная в которой храним предыдущее значение комплементарного фильтра
float Y_angle_comp; // Переменная в которой храним предыдущее значение комплементарного фильтра
float Z_angle_comp; // Переменная в которой храним предыдущее значение комплементарного фильтра

float X_angle_comp2; // Переменная в которой храним предыдущее значение комплементарного фильтра
float Y_angle_comp2; // Переменная в которой храним предыдущее значение комплементарного фильтра
float Z_angle_comp2; // Переменная в которой храним предыдущее значение комплементарного фильтра

float kalmAngleX;
float kalmAngleY;

void PrintRawData()
{
	Serial.print("RAW -> ");
	Serial.print(ax);   Serial.print(" , ");
	Serial.print(ay);   Serial.print(" , ");
	Serial.print(az);   Serial.print(" ;  !!  ;  ");
	//Serial.print(gx);   Serial.print(" , ");
	//Serial.print(gy);   Serial.print(" , ");
	//Serial.print(gz);   Serial.print(" ,  !!  ;  ");
	//Serial.print(temp9255);   Serial.print(" , ");
	Serial.print(" ");
}
void PrintData()
{
	Serial.print(" Data -> ");
	Serial.print(accX, 2);   Serial.print(" , ");
	Serial.print(accY, 2);   Serial.print(" , ");
	Serial.print(accZ, 2);   Serial.print(" ;  !!  ");
	Serial.print(gyroX, 2);   Serial.print(" , ");
	Serial.print(gyroY, 2);   Serial.print(" , ");
	Serial.print(gyroZ, 2);   Serial.print(" , ");
	Serial.println(" ");

}
void PrintAngle()
{
	Serial.print("Angle -> ");
	Serial.print(angleCompX2, 2);   Serial.print(" ");
	Serial.print(angleCompX, 2);   Serial.print(" ");
	Serial.print(kalmAngleX, 2);   Serial.print(" ");
	Serial.print(angleAccX, 1);   Serial.print(" ");
	Serial.print(angleGyroY, 1);   Serial.print(" ");
	
	Serial.print("  ");

	//Serial.print(angleCompY2, 2);   Serial.print(" ");
	//Serial.print(angleCompY, 2);   Serial.print(" ");
	//Serial.print(kalmAngleY, 2);   Serial.print(" ");
	//Serial.print(angleGyroX, 1);   Serial.print(" ");
	//Serial.print(angleAccY, 1);   Serial.print(" ");

	//Serial.print(angleGyroZ, 1);   Serial.print(" , ");
	Serial.println(" ");
}
void Init_9255()
{
	Serial.println(" ----------------------------------------------------------");
	Serial.println(" Start MPU9250...");
	Serial2.println(" Start Init_9255 ");

	set_TCA9548A(4);


	//pinMode(MPU9255_CS, OUTPUT);    // MPU9255 ChipSelect SPI
	//digitalWrite(MPU9255_CS, HIGH);

	uint8_t WIA_MPU;
	//WIA_MPU = ReadByte_SPI(MPU9255_CS, 0x75);     // Считываем значение регистра "Кто я"
	WIA_MPU = ReadByte_I2C(MPU9255_Address, 0x75);     // Считываем значение регистра "Кто я"
	Serial.print("WIA_MPU: "); Serial.print(WIA_MPU, HEX);

	angleGyroX = 0;           // Обнуляем начальные значения угла
	angleGyroY = 0;
	angleGyroZ = 0;

	//digitalWrite(MPU9255_CS,LOW);
	//uint8_t dump;
	//dump = SPI.transfer(0xF5); // READ(MSB=1) 0x80 or 0x75 -> 0xF5
	//Serial.print("dump: "); Serial.println(dump);
	//who_am_i = SPI.transfer(0);
	//digitalWrite(MPU9255_CS, HIGH);



	if (WIA_MPU == 0x68 | WIA_MPU == 0x71 | WIA_MPU == 0x73)          //71- 9050       73-9255
	{
		if (WIA_MPU == 0x68)  Serial.println(" Successfully connected to MPU6050");
		if (WIA_MPU == 0x71)  Serial.println(" Successfully connected to MPU9250");
		if (WIA_MPU == 0x73)  Serial.println(" Successfully connected to MPU9255");

		//	SPI.setClockDivider(SPI_CLOCK_DIV16);  // чтение SPI_CLOCK_DIV2  Устанавливаем скорость работы протокола SPI быстрее не поддерживает 9255

		WriteByte_I2C(MPU9255_Address, 0x6B, 0b10000000);                         // 107 регистр Reset MPU9255 
		//WriteByte_SPI(MPU9255_CS, 0x6B, 0b10000000);                         // 107 регистр Reset MPU9255 
		delay(10);																// После  ждем 1 миллисекунду

		//WriteByte_SPI(MPU9255_CS, 0x6A, 0b00010000);                         // 106 регистр Отключение работы по I2C и отставление только SPI
		delay(1);  // После  ждем 1 миллисекунду

		//WriteByte_SPI(MPU9255_CS, 0x6B, 0b00000001);                         // 107 регистр Clock Source  Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
		WriteByte_I2C(MPU9255_Address, 0x6B, 0b00000001);                         // 107 регистр Clock Source  Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
		delay(1);  // После  ждем 1 миллисекунду

		WriteByte_I2C(MPU9255_Address, 0x6C, 0b00000000);                         // 108 регистр Включаем гироскоп и акселерометр
		//WriteByte_SPI(MPU9255_CS, 0x6C, 0b00000000);                         // 108 регистр Включаем гироскоп и акселерометр
		delay(1);  // После  ждем 1 миллисекунду
																		// 25 регистр SMPLRT_DIV Устанавливаем делитель внутренней частоты 
		//WriteByte_SPI(MPU9255_CS, 0x19, 19);                         // устанавливаем HEX формате 3 в BIN (11) и получаем частоту 1 раз в 4 милисекунды или 250 Герц 
//		WriteByte_I2C(MPU9255_Address, 0x19, 0b00000111);                         // устанавливаем HEX формате 7 в BIN (111) и получаем частоту 1 раз в 8 милисекунды или 125 Герц 
		WriteByte_I2C(MPU9255_Address, 0x19, 4);                         // устанавливаем HEX формате 7 в BIN (111) и получаем частоту 1 раз в 8 милисекунды или 125 Герц 

		delay(1);  // После  ждем 1 миллисекунду 

		//WriteByte_SPI(MPU9255_CS, 0x1A, 0b00000000);    //   250  Bandwidth(Hz);     // 26 регистр CONFIG Устанавливаем ширину пропускания Гироскопа      
	  //	WriteByte_SPI(MPU9255_CS, 0x1A, 0b00000001);    //   184  Bandwidth(Hz);     // 26 регистр CONFIG Устанавливаем ширину пропускания Гироскопа
		//WriteByte_I2C(MPU9255_Address, 0x1A, 0b00000001);    //   184  Bandwidth(Hz);     // 26 регистр CONFIG Устанавливаем ширину пропускания Гироскопа
		//WriteByte_SPI(MPU9255_CS, 0x1A, 0b00000010);    //   92  Bandwidth(Hz);     // 26 регистр CONFIG Устанавливаем ширину пропускания Гироскопа
		//WriteByte_SPI(MPU9255_CS, 0x1A, 0b00000011);    //   41  Bandwidth(Hz);     // 26 регистр CONFIG Устанавливаем ширину пропускания Гироскопа
		WriteByte_I2C(MPU9255_Address, 0x1A, 0b00000011);    //   41  Bandwidth(Hz);     // 26 регистр CONFIG Устанавливаем ширину пропускания Гироскопа
		//WriteByte_I2C(MPU9255_Address, 0x1A, 0b00000100);    //   20  Bandwidth(Hz);     // 26 регистр CONFIG Устанавливаем ширину пропускания Гироскопа
		delay(1);

		//WriteByte_SPI(MPU9255_CS, 0x1B, 0b00000000);   gyro_delitel = 131;    //  250dps     // 27 регистр GYRO_CONFIG Устанавливаем чувствительность Гироскопа
		WriteByte_I2C(MPU9255_Address, 0x1B, 0b00000000);   gyro_delitel = 131;    //  250dps     // 27 регистр GYRO_CONFIG Устанавливаем чувствительность Гироскопа
	 // WriteByte_SPI(MPU9255_CS, 0x1B, 0b00001000);   gyro_delitel = 65.5;   //  500dps     // 27 регистр GYRO_CONFIG Устанавливаем чувствительность Гироскопа
	//	WriteByte_SPI(MPU9255_CS, 0x1B, 0b00010000);   gyro_delitel = 32.8;   //  1000dps    // 27 регистр GYRO_CONFIG Устанавливаем чувствительность Гироскопа
		//WriteByte_I2C(MPU9255_Address, 0x1B, 0b00010000);   gyro_delitel = 32.8;   //  1000dps    // 27 регистр GYRO_CONFIG Устанавливаем чувствительность Гироскопа
	 // WriteByte_SPI(MPU9255_CS, 0x1B, 0b00011000);   gyro_delitel = 16.4;     //  2000dps      // 27 регистр GYRO_CONFIG Устанавливаем чувствительность Гироскопа      
		delay(1);


		//WriteByte_SPI(MPU9255_CS, 0x1C, 0b00000000);   acc_delitel = 16384;   //  +-2G      // 28 регистр ACCEL_CONFIG Устанавливаем чувствительность Аксельрометр
		WriteByte_I2C(MPU9255_Address, 0x1C, 0b00000000);   acc_delitel = 16384;   //  +-2G      // 28 регистр ACCEL_CONFIG Устанавливаем чувствительность Аксельрометр
		//WriteByte_SPI(MPU9255_CS, 0x1C, 0b00001000);   acc_delitel = 8192;    //  +-4G      // 28 регистр ACCEL_CONFIG Устанавливаем чувствительность Аксельрометр    
		//WriteByte_I2C(MPU9255_Address, 0x1C, 0b00001000);   acc_delitel = 8192;    //  +-4G      // 28 регистр ACCEL_CONFIG Устанавливаем чувствительность Аксельрометр    
		//WriteByte_SPI(MPU9255_CS,0x1C,0b00010000);   acc_delitel = 4096;    //  +-8G      // 28 регистр ACCEL_CONFIG Устанавливаем чувствительность Аксельрометр   
		//WriteByte_SPI(MPU9255_CS,0x1C,0b00011000);     acc_delitel = 2048;  //  +-16G     // 28 регистр ACCEL_CONFIG Устанавливаем чувствительность Аксельрометр

		delay(1);  // После  ждем 1 миллисекунду

	  //  WriteByte_SPI(MPU9255_CS, 0x1D, 0b00001000);    //   460  Bandwidth(Hz);     // 29 регистр ACCEL_CONFIG2 Устанавливаем ширину пропускания Аксельрометр
	//	WriteByte_SPI(MPU9255_CS, 0x1D, 0b00001001);    //   184  Bandwidth(Hz);     // 29 регистр ACCEL_CONFIG2 Устанавливаем ширину пропускания Аксельрометр      
		//WriteByte_I2C(MPU9255_Address, 0x1D, 0b00001001);    //   184  Bandwidth(Hz);     // 29 регистр ACCEL_CONFIG2 Устанавливаем ширину пропускания Аксельрометр      
		//WriteByte_SPI(MPU9255_CS, 0x1D, 0b00001010);    //   92  Bandwidth(Hz);     // 29 регистр ACCEL_CONFIG2 Устанавливаем ширину пропускания Аксельрометр      
	//	WriteByte_SPI(MPU9255_CS, 0x1D, 0b00001011);    //   41  Bandwidth(Hz);     // 29 регистр ACCEL_CONFIG2 Устанавливаем ширину пропускания Аксельрометр      
		WriteByte_I2C(MPU9255_Address, 0x1D, 0b00001011);    //   41  Bandwidth(Hz);     // 29 регистр ACCEL_CONFIG2 Устанавливаем ширину пропускания Аксельрометр      
		//WriteByte_I2C(MPU9255_Address, 0x1D, 0b00001100);    //   20  Bandwidth(Hz);     // 29 регистр ACCEL_CONFIG2 Устанавливаем ширину пропускания Аксельрометр      

		delay(1);  // После  ждем 1 миллисекунду




		WriteByte_I2C(MPU9255_Address, 0x37, 0b00010000);         // 55 регистр INT_PIN_CFG Устанавливаем очищение статура прерывания при любом чтении из регистра данных
		//WriteByte_SPI(MPU9255_CS, 0x37, 0b00000000);         // 55 регистр INT_PIN_CFG Устанавливаем очищение статура прерывания при любом чтении из регистра данных

		delay(1);

		//WriteByte_SPI(MPU9255_CS, 0x38, 0b00000001);         // 56 регистр INT_PIN_CFG Включаем прерывание на пине
		WriteByte_I2C(MPU9255_Address, 0x38, 0b00000001);         // 56 регистр INT_PIN_CFG Включаем прерывание на пине
		delay(1);
		Serial.println(" attachInterrupt Enable!!!");

		// Set by pass mode for the magnetometers
		WriteByte_I2C(MPU9255_Address, 0x37, 0x02);		 //Включаем доступ к магнетрометру по I2C по этим же контактам. Типа соединяем внутреннюю шину и внешнюю .


		//SPI.setClockDivider(SPI_CLOCK_DIV2);  // SPI_CLOCK_DIV2       Устанавливаем скорость SPI	чтение SPI_CLOCK_DIV2 делитель 2 к частоте адруино 16 = 8 Мгерц

	}
	else
	{
		Serial.println("Failed to Connect to MPU9250 !!!!!!!!!!!!!!");
		delay(1000);
	}


}

float getComplementarnX(float angleAccX, float angleGyro_Delta_Y, float koef)       //Простейший комплементарный фильтр
{
	X_angle_comp2 = ((1 - koef) * (X_angle_comp2 + angleGyro_Delta_Y)) + (koef * angleAccX);
	return X_angle_comp2;
}

double getCompAngleX(float comp_AngleX, float comp_GyroY, float dt)       //Простейший комплементарный фильтр
{
	//a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc 
	float coefficient = 0.02;
	X_angle_comp = ((1 - coefficient) * (X_angle_comp + comp_GyroY * dt)) + (coefficient * comp_AngleX);
	return X_angle_comp;
}


double getCompAngleY(float comp_AngleY, float comp_GyroX, float dt)       //Простейший комплементарный фильтр
{
	//a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc 
	float coefficient = 0.02;
	Y_angle_comp = ((1 - coefficient) * (Y_angle_comp + comp_GyroX * dt)) + (coefficient * comp_AngleY);

	return Y_angle_comp;
}
float getComplementarnY(float angleAccY, float angleGyro_Delta_X, float koef)       //Простейший комплементарный фильтр
{
	Y_angle_comp2 = ((1 - koef) * (Y_angle_comp2 + angleGyro_Delta_X)) + (koef * angleAccY);
	return Y_angle_comp2;
}

double getCompAngleZ(float magD, float comp_GyroZ, float dt)       //Простейший комплементарный фильтр
{
	//a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc 
	float coefficient = 0.001;

	//Serial.print(" magD= ");	Serial.print(magD);
	float gyroD = Z_angle_comp + (comp_GyroZ * dt);

	if (gyroD < 0) { gyroD = gyroD + 360; }
	if (gyroD > 360) { gyroD = gyroD - 360; }

	//Serial.print(" g1= ");	Serial.print(gyroD);

	//Переход через 360 и привелдение к одной шкале magD

	if (gyroD < 10 && magD > 350)      // Если в разных сторонах от 360 градусов
	{
		gyroD = gyroD + 360;       // тогда добавляем 360 и они попучаются в одной области
	}
	if (gyroD > 350 && magD < 10)      // Если в разных сторонах от 360 градусов
	{
		gyroD = gyroD - 360;       // тогда добавляем 360 и они попучаются в одной области
	}

	//Serial.print(" g2= ");	Serial.print(gyroD);


	Z_angle_comp = ((1 - coefficient) * gyroD) + (coefficient * magD);

	//Serial.print(" Z1= ");	Serial.print(Z_angle_comp);

	if (Z_angle_comp < 0) { Z_angle_comp = Z_angle_comp + 360; }
	if (Z_angle_comp > 360) { Z_angle_comp = Z_angle_comp - 360; }

	//Serial.print(" Z_angle_comp= ");	Serial.println(Z_angle_comp);

	return Z_angle_comp;
}

/*
void Read_9255()  // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	//Serial.print("+");

//	SPI.setClockDivider(SPI_CLOCK_DIV2);  // чтение SPI_CLOCK_DIV2 делитель 2 к частоте адруино 16 = 8 Мгерц

	//SPI.beginTransaction(settings2);
	digitalWrite(MPU9255_CS, LOW);
	int8_t registr = 0x3B;			  // Начало регистров последовательно 14 штук	начиная с этого
	registr = registr | (1 << 7);   //   READ(MSB=1) 0x80 or 0x75 -> 0xF5   // Добавляем старший бит так как это чтение
	SPI.transfer(registr);
	//Serial.print("= ");Serial.println(micros());
	for (uint8_t i = 0; i < 14; i++)   // Считываем 14 байт uint16_t - это 7 двухбайтный значений uint16_t
	{
		buffer_acc_gyro[i] = SPI.transfer(0x00);
	}
	digitalWrite(MPU9255_CS, HIGH);
	//SPI.endTransaction();

	ax = (((int16_t)buffer_acc_gyro[0]) << 8) | buffer_acc_gyro[1];
	ay = (((int16_t)buffer_acc_gyro[2]) << 8) | buffer_acc_gyro[3];
	az = (((int16_t)buffer_acc_gyro[4]) << 8) | buffer_acc_gyro[5];
	temp9255 = (((int16_t)buffer_acc_gyro[6]) << 8) | buffer_acc_gyro[7];
	gx = (((int16_t)buffer_acc_gyro[8]) << 8) | buffer_acc_gyro[9];
	gy = (((int16_t)buffer_acc_gyro[10]) << 8) | buffer_acc_gyro[11];
	gz = (((int16_t)buffer_acc_gyro[12]) << 8) | buffer_acc_gyro[13];
	//Temperatura9255 = (temp9255 - 0) / 333.87 + 21;     //Расчет температуры самого чипа по Цельсию, а НЕ температуры воздуха

	accX = ((float)ax) / acc_delitel;
	accY = ((float)ay) / acc_delitel;
	accZ = ((float)az) / acc_delitel;
	gyroX = ((float)gx) / gyro_delitel;
	gyroY = ((float)gy) / gyro_delitel;
	gyroZ = ((float)gz) / gyro_delitel;
	//Учитываем калибровку 
	accX -= accXoffset;
	accY -= accYoffset;
	accZ -= accZoffset;
	//Учитываем калибровку 
	gyroX -= gyroXoffset;
	gyroY -= gyroYoffset;
	gyroZ -= gyroZoffset;
}
*/
void Read_9255_I2C()  // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	//Serial2.print("/");

	//Serial2.print("WIA_MPU: "); Serial2.print(ReadByte_I2C(MPU9255_Address, 0x75), HEX);
	Wire.beginTransmission(MPU9255_Address);
	int8_t registr = 0x3B;			  // Начало регистров последовательно 14 штук	начиная с этого
	Wire.write(registr);
	byte reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print("!!! 9255 Write Mistake reza = ");Serial.println(reza);
		Serial2.print("!!! 9255 Write Mistake reza = ");Serial2.println(reza);

	};

	uint8_t buffer_acc_gyroTemp[14];			 // Буфер для считывания данных аксельрометра  временный
	byte count_byte = 14;			  // число запрашиваемых байт

	int rezb = Wire.requestFrom(MPU9255_Address, count_byte);
	//Serial2.print(rezb);
	if (rezb == count_byte)         // Если вернулось  столько сколько просили
	{
		for (byte i = 0;i < count_byte; i++)   // Считываем 14 байт uint16_t - это 7 двухбайтный значений uint16_t если такие данные есть
		{
			buffer_acc_gyro[i] = Wire.read();
		}
	}
	else
	{
		Serial.print("!!! 9255 Write Mistake rezb = ");Serial.println(rezb);
		Serial2.print("!!! 9255 Write Mistake rezb = ");Serial2.println(rezb);

	}


	ax = (((int16_t)buffer_acc_gyro[0]) << 8) | buffer_acc_gyro[1];
	ay = (((int16_t)buffer_acc_gyro[2]) << 8) | buffer_acc_gyro[3];
	az = (((int16_t)buffer_acc_gyro[4]) << 8) | buffer_acc_gyro[5];
	temp9255 = (((int16_t)buffer_acc_gyro[6]) << 8) | buffer_acc_gyro[7];
	gx = (((int16_t)buffer_acc_gyro[8]) << 8) | buffer_acc_gyro[9];
	gy = (((int16_t)buffer_acc_gyro[10]) << 8) | buffer_acc_gyro[11];
	gz = (((int16_t)buffer_acc_gyro[12]) << 8) | buffer_acc_gyro[13];
	//Temperatura9255 = (temp9255 - 0) / 333.87 + 21;     //Расчет температуры самого чипа по Цельсию, а НЕ температуры воздуха

	accX = ((float)ax) / acc_delitel;
	accY = ((float)ay) / acc_delitel;
	accZ = ((float)az) / acc_delitel;
	gyroX = ((float)gx) / gyro_delitel;
	gyroY = ((float)gy) / gyro_delitel;
	gyroZ = ((float)gz) / gyro_delitel;
	//Учитываем калибровку 
	accX -= accXoffset;
	accY -= accYoffset;
	accZ -= accZoffset;
	//Учитываем калибровку 
	gyroX -= gyroXoffset;
	gyroY -= gyroYoffset;
	gyroZ -= gyroZoffset;

	//uint8_t i = 0;

	//while (Wire.available())    // ведомое устройство может послать меньше, чем запрошено
	//{
	//	buffer_acc_gyroTemp[i] = Wire.read();
	//	i++;
	//}

	//for (;i < 14 && (Wire.available() > 0); i++)   // Считываем 14 байт uint16_t - это 7 двухбайтный значений uint16_t если такие данные есть
	//{
	//	buffer_acc_gyroTemp[i] = Wire.read();
	//}
	//Serial2.println("\ "); 
//	Serial2.print(i);

	//if (i == 14)       // Если прочитали все 14 байт полностью и ничего не пропало тогда из врменного массива переписываем в нормальный
	//{
	//	for (int y = 0; y < 14; y++) {	buffer_acc_gyro[y] = buffer_acc_gyroTemp[y]; }
	//}
	//else
	//{
	//	Serial2.println(" BLAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
	//	for (int y = 0; y < 14; y++) 
	//	{ 
	//		Serial2.print(buffer_acc_gyro[y]);
	//		Serial2.print("new->");
	//		Serial2.println(buffer_acc_gyroTemp[y]);
	//		//buffer_acc_gyro[y] = 0;
	//	}

	//}

	//Serial.print("T");Serial.println(micros());
	//Serial.print(".");
}



void Update_9255()
{

	//Вычисляем углы

	angleAccX = atan2(accX, accZ) * RAD_TO_DEG;
	angleAccY = atan2(accY, accZ) * RAD_TO_DEG;


	//angleAccX = atan2(accX, accZ + abs(accY)) * RAD_TO_DEG;		   //Второй вариант расчета учитывает одноврмеменные наклонения
	//angleAccY = atan2(accY, accZ + abs(accX)) * RAD_TO_DEG;		   // Вадим считает что он не правильный
/*
//---------------------- НАКЛОН ДАТЧИКА КВЕРХ НОГАМИ------------------------------------------------------------------
	if (angleAccX < 0) { angleAccX = angleAccX + 180; }
	else { angleAccX = -180 + angleAccX; }  // Перевод угла наклона с учётом того как установлена микросхема на машинке
	if (angleAccY < 0) { angleAccY = -angleAccY - 180; }
	else { angleAccY = 180 - angleAccY; }  // Перевод угла наклона с учётом того как установлена микросхема на машинке
	angleAccX = -angleAccX;
	angleAccY = -angleAccY;
//----------------------------------------------------------------------------------------
*/
	//Скорость по гироскопу умножаем на прошедшее время и получаем угол на который за это время отклонился
	angleGyro_Delta_X = gyroX * intervalAccGyro;
	angleGyro_Delta_Y = -gyroY * intervalAccGyro;
	angleGyro_Delta_Z = gyroZ * intervalAccGyro;

	angleGyroX += angleGyro_Delta_X;			//Складываем угол и предыдущее значение
	angleGyroY += angleGyro_Delta_Y;
	angleGyroZ += angleGyro_Delta_Z;

	//if (angleGyroZ > 360) { angleGyroZ = angleGyroZ - 360; }	  // Преобразование в 360 градусов
	//if (angleGyroZ < 0) { angleGyroZ = angleGyroZ + 360; }


	// Вычисляем углы по аксельрометру и гироскопу
	//angleCompX = getCompAngleX(angleAccX, -gyroY, intervalAccGyro);		  //Вадим говорит что оси икс по аксельрометру и игрек по гироскопу
	//angleCompY = getCompAngleY(angleAccY, gyroX, intervalAccGyro);

	// Вычисляем углы по аксельрометру и гироскопу
	angleCompX = getCompAngleX(angleAccX, -gyroY, intervalAccGyro);
	angleCompX2 = getComplementarnX(angleAccX, angleGyro_Delta_Y, 0.01);

	angleCompY = getCompAngleY(angleAccY, gyroX, intervalAccGyro);
	angleCompY2 = getComplementarnY(angleAccY, angleGyro_Delta_X, 0.01);

//	angleCompZ = getCompAngleZ(magD, gyroZ, intervalAccGyro);
	//ВЫчисляем угол поворота используя магнетрометр и гироскоп
	//myAHRS.compAngleZ = myAHRS.getCompAngleZ(magD, -gyroZ, intervalAccGyro);
	//myAHRS.compAngleZ = myAHRS.getCompAngleZ(magD, angleGyroZ, intervalAccGyro);

	kalmAngleX = kalmanX.getAngle(angleAccX, -gyroY, intervalAccGyro); // Calculate the angle using a Kalman filter X
	kalmAngleY = kalmanY.getAngle(angleAccY, gyroX, intervalAccGyro); // Calculate the angle using a Kalman filter Y
	//	Filter_Complementary();            // Фильтруем значения

	//Serial2.println("UpGood");

}

void ReadCalibrovka_AccGyro()
{
	if (eeprom_read_byte(32) == 1)   // Если есть поправочные значения то считываем их
	{
		Serial.println(" Read Colibrovka from EEPROM ");
		accXoffset = eeprom_read_float(34);
		accYoffset = eeprom_read_float(38);
		accZoffset = eeprom_read_float(42);
		gyroXoffset = eeprom_read_float(46);
		gyroYoffset = eeprom_read_float(50);
		gyroZoffset = eeprom_read_float(54);
		Serial.print("AX : ");Serial.println(accXoffset, 6);
		Serial.print("AY : ");Serial.println(accYoffset, 6);
		Serial.print("AZ : ");Serial.println(accZoffset, 6);
		Serial.print("GX : ");Serial.println(gyroXoffset, 6);
		Serial.print("GY : ");Serial.println(gyroYoffset, 6);
		Serial.print("GZ : ");Serial.println(gyroZoffset, 6);
		Serial.println(" ----------------------------------------------");
		StatusColibrAccGyro = true;

	}
}



void  int5_func()         // MPU9255 Функция прерывания 9255  5 прерывание 18 пин  
{
	//PIN_A_2_HIGH;
	//Serial.println("/");
	if (StatusColibrAccGyro == true)    // Если уже произвели калибровку
	{
		now_time = micros();
		intervalAccGyro = (now_time - pre_intervalAccGyro)*0.000001;			// Измеряем время прошедшее с предудущего считывания данных гироскопа в секундах
		pre_intervalAccGyro = now_time;
		flag_DataReady_AccGyro = true;     // Взводим флаг готовности данных для основного потока

	//	Serial.print("T");Serial.println(micros());
		//Serial.print(".");

	}
	//PIN_A_2_LOW;

	
}
void PrintGyroTest()
{
	Serial.print(angleGyroX);   Serial.print("  ");
	Serial.print(angleGyroY);   Serial.print("  ");
	//Serial.print(angleGyroZ);   Serial.print("  ");
	Serial.println(" ");
}



void Loop_9255()
{
}



void Calibrovka_AccGyro()
{

	float ax = 0, ay = 0, az = 0;
	float gx = 0, gy = 0, gz = 0;
	int ii = 2000;       // длительность калибровки
	accXoffset = 0;
	accYoffset = 0;
	accZoffset = 0;

	gyroXoffset = 0;
	gyroYoffset = 0;
	gyroZoffset = 0;

	Serial.println("======================= calculate  offsets =================");

	for (int i = 0; i < ii; i++)
	{
		// Опрашиваем гироскоп, акселерометр и магнетрометр
		Read_9255_I2C();            //Считываем все 7 значений из аксель-гироскопа
		//Update_9255();
		//Serial.print(".");
		//Serial.print(" i = ");   
		//Serial.println(i);
		ax += accX; //Serial.print(" AX+ = ");    Serial.print(ax);
		ay += accY;// Serial.print(" AY+ = ");    Serial.print(ay);
		az += accZ; //Serial.print(" AZ+ = ");    Serial.print(az);

		gx += gyroX;// Serial.print(" GX+ = ");    Serial.print(gx);
		gy += gyroY; //Serial.print(" GY+ = ");    Serial.print(gy);
		gz += gyroZ;// Serial.print(" GZ+ = ");    Serial.println(gz);
		delay(2);
	}
	Serial.println("End calculate offsets. ");

	accXoffset = ax / ii;
	accYoffset = ay / ii;
	//accZoffset = (az / ii ) + 1;  // Приводим не к нулю, а к минус единице, 1g земная гравитация Вычисления зависят от того как установлена микросхема 
	accZoffset = (az / ii) - 1;  // Приводим не к нулю, а к единице, 1g земная гравитация Вычисления зависят от того как установлена микросхема 

	gyroXoffset = gx / ii ;
	gyroYoffset = gy / ii ; // Тут можно вручную добавить поправку к автоматическому определению
	gyroZoffset = gz / ii ;

	angleGyroX = 0;           // Обнуляем начальные значения угла
	angleGyroY = 0;
	angleGyroZ = 0;

	eeprom_write_byte(32, 1);     // записываем признак что есть данные колибровки в 10 адрес
	Serial.print("Flag ACC_GYRO: "); Serial.println(eeprom_read_byte(32));

	Serial.println("==================ACC======================");

	Serial.print("X : ");Serial.println(accXoffset, 6);
	eeprom_write_float(34, accXoffset);
	Serial.print("accX offset : "); Serial.println(eeprom_read_float(34), 6);

	Serial.print("Y : ");Serial.println(accYoffset, 6);
	eeprom_write_float(38, accYoffset);
	Serial.print("accY offset : "); Serial.println(eeprom_read_float(38), 6);

	Serial.print("Z : ");Serial.println(accZoffset, 6);
	eeprom_write_float(42, accZoffset);
	Serial.print("accZ offset : "); Serial.println(eeprom_read_float(42), 6);

	Serial.println("==================GYRO=====================");
	Serial.print("X : ");Serial.println(gyroXoffset, 6);
	eeprom_write_float(46, gyroXoffset);
	Serial.print("gyroX offset : "); Serial.println(eeprom_read_float(46), 6);

	Serial.print("Y : ");Serial.println(gyroYoffset, 6);
	eeprom_write_float(50, gyroYoffset);
	Serial.print("gyroY offset : "); Serial.println(eeprom_read_float(50), 6);

	Serial.print("Z : ");Serial.println(gyroZoffset, 6);
	eeprom_write_float(54, gyroZoffset);
	Serial.print("gyroZ offset : "); Serial.println(eeprom_read_float(54), 6);

	Serial.println("========================================");

	StatusColibrAccGyro = true;
}




void PrintAngleCompFiltr()
{
	//Serial.print("AngleCompFilter -> ");
	Serial.print(angleCompX, 2);   Serial.print(" , ");
	//Serial.print(BNO055_EulerAngles.x, 2);   Serial.print(" , ");

	Serial.print(angleCompY, 2);   Serial.print(" , ");
	//Serial.print(BNO055_EulerAngles.y, 2);   Serial.print(" , ");
	Serial.println(" ");

}
void PrintAngleTest()
{
	//Serial.print(angleAccX);   Serial.print(" , ");
	//Serial.print(angleGyroY);   Serial.print(" , ");
	//Serial.print(angleCompX);   Serial.print(" , ");
	//Serial.print(BNO055_EulerAngles.x);   Serial.print(" , ");

	//Serial.print(angleAccY);   Serial.print(" , ");
	//Serial.print(angleGyroX);   Serial.print(" , ");
	//Serial.print(angleCompY);   Serial.print(" , ");
	//Serial.print(BNO055_EulerAngles.y);   Serial.print(" , ");


	Serial.print(magD);   Serial.print(" , ");
	Serial.print(angleGyroZ);   Serial.print(" , ");
	Serial.print(angleCompZ);   Serial.print(" , ");
	//Serial.print(BNO055_EulerAngles.z);   Serial.print(" , ");
	//Serial.print(BNO055_EulerAngles.my_z);   Serial.print(" , ");
	//Serial.print(BNO055_EulerAngles.delta_z);   Serial.print(" , ");



	Serial.println(" , ");

}


void PrintMag()
{
	//Serial.print("Magnetrometr -> ");
	//Serial.print(magX);   Serial.print(" , ");
	//Serial.print(magY);   Serial.print(" , ");
	//Serial.print(magZ);   Serial.print(" , ");
	//Serial.print(magD);   Serial.print(" , ");
	//Serial.println(" , ");
}

void PrintDirection()
{
	Serial.print(" Dir-> ");
	Serial.print(magD);   	  Serial.print(" ");

	//Serial.print(gyroZ);   	  Serial.print(" , ");
	Serial.print(angleGyroZ, 4);   Serial.print(" ");

	//Serial.print(myAHRS.compAngleZ, 0);

	Serial.println(" ");
}
void Setup_9250()
{
	Init_9255();
	//Calibrovka_AccGyro();		  // Если нужно то колибруем аксельрометр-гироскоп
	ReadCalibrovka_AccGyro();	  // Если уже откалиброано то просто считываем значения
	//   Включаем прерывания последними
	attachInterrupt(0, int5_func, RISING);   // Запускаем Прерывание для 9255 номер 3 на 18 пине.
	Read_9255_I2C();            //Считываем все 7 значений из аксель-гироскопа первый раз
	Update_9255();

	X_angle_comp = angleAccX;	//Первый раз комплементарному значению присваиваем значение по аксельрометру
	X_angle_comp2 = angleAccX;	//Первый раз комплементарному значению присваиваем значение по аксельрометру
	kalmanX.setAngle(angleAccX);

	//Y_angle_comp = angleAccY;	//Первый раз комплементарному значению присваиваем значение по аксельрометру
	//Y_angle_comp2 = angleAccY;	//Первый раз комплементарному значению присваиваем значение по аксельрометру
	//kalmanY.setAngle(angleAccY);

	Serial.println("AttachInterrupt Start");
}