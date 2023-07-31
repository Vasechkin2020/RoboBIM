#ifndef I2C_MY_H
#define I2C_MY_H



int16_t uint8ToUint16(uint8_t Hbyte, uint8_t Lbyte) // Из двух байт получаем двухбайтное число. не перепутать старший и младший байт. так как бывает разная последовательность регистров
{
  return ((int16_t)Hbyte << 8) | Lbyte;
}

void WriteByte_I2C(uint8_t address, int8_t registr, uint8_t data)
{
  Wire.beginTransmission(address);
  Wire.write(registr);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t ReadByte_I2C(uint8_t address, int8_t registr)
{
  Wire.beginTransmission(address);
  Wire.write(registr);
  byte reza = Wire.endTransmission();
  if (reza != 0)
  {
    Serial.print("!!! ReadByte_I2C_WriteMistake reza = ");
    Serial.println(reza);
  };
  byte rezb = Wire.requestFrom(address, (uint8_t)1);
  if (rezb == 1)
  {
    uint8_t data = Wire.read(); //read one byte of data
    return data;
  }
  else
  {
    Serial.print("!!! ReadByte_I2C_WriteMistake rezb = ");
    Serial.println(rezb);
    return 0;
  }
}
uint16_t ReadWord_I2C(uint8_t address, int8_t registr)
{
  Wire.beginTransmission(address);
  Wire.write(registr);
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)2);
  uint8_t Hbyte = Wire.read(); //read High byte of data
  uint8_t Lbyte = Wire.read(); //read Low byte of data
  return ((int16_t)Hbyte << 8) | Lbyte;
  ; // Сдвигаем старший байт влево и добавляем младший байт
}

// Функция сканирования I2C устройств
void scanI2C()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning I2C...");

  nDevices = 0;
  for (address = 8; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
Serial.println("End scanning I2C...");

}

void scaner_i2c() // ---------------- Определение адресов устройств -----------------
{
  // put your setup code here, to run once:
  while (!Serial)
  {
  }
  Serial.println();
  Serial.println("I2C scanner. Scanning ...");
  byte count = 0;
  //Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("Found address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");
      count++;
      delay(1); // maybe unneeded?
    }           // end of good response
  }             // end of for loop
  Serial.println("Done.");
  Serial.print("Found ");
  Serial.print(count, DEC);
  Serial.println(" device(s).");
}

void Write8_I2C(uint8_t address, int8_t registr, uint8_t data)
{
  Wire.beginTransmission(address);
  Wire.write(registr);
  Wire.write(data);
  int reza = Wire.endTransmission();
  if (reza != 0)
  {
    Serial.print(" Write8_I2C Mistake reza= ");
    Serial.println(reza);
  }
}
void Write16_I2C(uint8_t address, int8_t registr, int16_t data)
{
  Wire.beginTransmission(address);
  Wire.write(registr);
  Wire.write(data >> 8);
  Wire.write(data & 0xFF);
  int reza = Wire.endTransmission();
  if (reza != 0)
  {
    Serial.print(" Write16_I2C Mistake reza= ");
    Serial.println(reza);
  }
}
void Write32_I2C(uint8_t address, int8_t registr, int32_t data)
{
  Wire.beginTransmission(address);
  Wire.write(registr);
  Wire.write(data >> 24);
  Wire.write(data >> 16);
  Wire.write(data >> 8);
  Wire.write(data & 0xFF);
  int reza = Wire.endTransmission();
  if (reza != 0)
  {
    Serial.print(" Write32F_I2C Mistake reza= ");
    Serial.println(reza);
  }
}
void Write32F_I2C(uint8_t address, int8_t registr, float data)
{
  //Serial.print(" float = "); Serial.println(data, BIN);

  uint8_t *pB = (uint8_t *)(&data);
  uint8_t LL_byte = pB[0];
  uint8_t L_byte = pB[1];
  uint8_t H_byte = pB[2];
  uint8_t HH_byte = pB[3];

  Wire.beginTransmission(address);
  Wire.write(registr);
  Wire.write(LL_byte); //    Serial.print("LL_byte: "); Serial.println(LL_byte,BIN);
  Wire.write(L_byte);  //    Serial.print("L_byte: "); Serial.println(L_byte,BIN);
  Wire.write(H_byte);  //    Serial.print("H_byte: "); Serial.println(H_byte,BIN);
  Wire.write(HH_byte); //  Serial.print("HH_byte: "); Serial.println(HH_byte,BIN);
  int reza = Wire.endTransmission();
  if (reza != 0)
  {
    Serial.print(" Write32F_I2C float Mistake reza= ");
    Serial.println(reza);
  }
}
uint16_t ReadWord_I2C_LE(uint8_t address, int8_t registr) // Обратный порядок байт !!!!!!!!!!!!!!
{
  Wire.beginTransmission(address);
  Wire.write(registr);
  int reza = Wire.endTransmission();
  if (reza != 0)
  {
    Serial.print(" ReadWord_I2C_LE Mistake reza= ");
    Serial.println(reza);
  }
  int rezb = Wire.requestFrom(address, (uint8_t)2);
  if (rezb == 2)
  {
    uint8_t Lbyte = Wire.read(); //read Low  byte of data
    uint8_t Hbyte = Wire.read(); //read High byte of data
    return ((int16_t)Hbyte << 8) | Lbyte;
    ; // Сдвигаем старший байт влево и добавляем младший байт
  }
  else
  {
    Serial.print(" ReadWord_I2C_LE Mistake rezb= ");
    Serial.println(rezb);
  }
  return 0;
}

#endif