// Distributed with a free-will license.
// Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
// BMP280
// This code is designed to work with the BMP280_I2CS I2C Mini Module available from ControlEverything.com.
// https://www.controleverything.com/content/Barometer?sku=BMP280_I2CSs#tabs-0-product_tabset-2

#include<Wire.h>

// BMP280 I2C address is 0x76(108)
#define Addr 0x76

void setup()
{
  // Initialise I2C communication as MASTER
  Wire.begin();
  // Initialise Serial communication, set baud rate = 9600
  Serial.begin(9600);
}

void loop()
{
  unsigned int b1[24];
  unsigned int data[8];
  for (int i = 0; i < 24; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write((136 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);

    // Read 1 byte of data
    if (Wire.available() == 1)
    {
      b1[i] = Wire.read();
    }
  }
  // Convert the data
  // temp coefficients
  unsigned int dig_T1 = (b1[0] & 0xFF) + ((b1[1] & 0xFF) * 256);
  int dig_T2 = b1[2] + (b1[3] * 256);
  int dig_T3 = b1[4] + (b1[5] * 256);

  // pressure coefficients
  unsigned int dig_P1 = (b1[6] & 0xFF) + ((b1[7] & 0xFF) * 256);
  int dig_P2 = b1[8] + (b1[9] * 256);
  int dig_P3 = b1[10] + (b1[11] * 256);
  int dig_P4 = b1[12] + (b1[13] * 256);
  int dig_P5 = b1[14] + (b1[15] * 256);
  int dig_P6 = b1[16] + (b1[17] * 256);
  int dig_P7 = b1[18] + (b1[19] * 256);
  int dig_P8 = b1[20] + (b1[21] * 256);
  int dig_P9 = b1[22] + (b1[23] * 256);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select control measurement register
  Wire.write(0xF4);
  // Normal mode, temp and pressure over sampling rate = 1
  Wire.write(0x27);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select config register
  Wire.write(0xF5);
  // Stand_by time = 1000ms
  Wire.write(0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();

  for (int i = 0; i < 8; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write((247 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);

    // Read 1 byte of data
    if (Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }

  // Convert pressure and temperature data to 19-bits
  long adc_p = (((long)(data[0] & 0xFF) * 65536) + ((long)(data[1] & 0xFF) * 256) + (long)(data[2] & 0xF0)) / 16;
  long adc_t = (((long)(data[3] & 0xFF) * 65536) + ((long)(data[4] & 0xFF) * 256) + (long)(data[5] & 0xF0)) / 16;

  // Temperature offset calculations
  double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
                 (((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
  double t_fine = (long)(var1 + var2);
  double cTemp = (var1 + var2) / 5120.0;
  double fTemp = cTemp * 1.8 + 32;

  // Pressure offset calculations
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
  var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
  double p = 1048576.0 - (double)adc_p;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double) dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double) dig_P8) / 32768.0;
  double pressure = (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100;

  // Output data to serial monitor
  Serial.print("Pressure : ");
  Serial.print(pressure);
  Serial.println(" hPa");
  Serial.print("Temperature in Celsius : ");
  Serial.print(cTemp);
  Serial.println(" C");
  Serial.print("Temperature in Fahrenheit : ");
  Serial.print(fTemp);
  Serial.println(" F");
  delay(1000);
}
