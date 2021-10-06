/****************************************************************
 * Example5_DualSPITest.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data on two IMUs over SPI
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.
#define CS_PIN_1 10   // Which pin you connect CS to for Sensor 1

ICM_20948_SPI myICM1; // Create an ICM_20948_SPI object

void setup()
{

  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };

  SPI_PORT.begin();

//  myICM1.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

    myICM1.begin(CS_PIN_1, SPI_PORT);

    SERIAL_PORT.print(F("Initialization of sensor 1 returned: "));
    SERIAL_PORT.println(myICM1.statusString());
    if (myICM1.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  SERIAL_PORT.print("Data Start");

}

bool dataSeen = false;

void loop()
{

  if (myICM1.dataReady())
  {
    myICM1.getAGMT(); // The values are only updated when you call 'getAGMT'
//    SERIAL_PORT.print("\nSensor 1: ");
//    SERIAL_PORT.print("Raw: ");
//    printRawAGMT( myICM1.agmt);
//    SERIAL_PORT_ECHO.print("Scaled: ");
//    SERIAL_PORT.print(myICM1.accX());
//    printScaledAGMT(&myICM1);
    sendPacket(&myICM1);
    
    dataSeen = true;
  }
  if (dataSeen)
  {
    delay(30);
    dataSeen = false;
  }
  else
  {
//    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }
}

void sendPacket(ICM_20948_SPI* sensor) {
  uint8_t dataBuffer[18];
  uint8_t twoByteBuffer[2];
  float measurements[] = {sensor->accX(), sensor->accY(), sensor->accZ(), 
                          sensor->gyrX(), sensor->gyrY(), sensor->gyrZ(),
                          sensor->magX(), sensor->magY(), sensor->magZ()};
  for (int i = 0; i < sizeof(measurements)/sizeof(measurements[0]); i++) {
    floatToBytes(measurements[i], twoByteBuffer);
    memcpy(dataBuffer + (2 * i), twoByteBuffer, 2);
  }
  SERIAL_PORT.write(dataBuffer, sizeof(dataBuffer)/sizeof(dataBuffer[0]));
}

// Adapted from Mohsen Sarkars answer at https://stackoverflow.com/a/37761168 (CC BY-SA 3.0)
void floatToBytes(float twoByteFloat, uint8_t* a_twoByteBuffer) {
  int32_t fbits;
  memcpy(&fbits, &twoByteFloat, sizeof(fbits));
  int32_t sign = fbits >> 16 & 0x8000;
  int32_t val = (fbits & 0x7fffffff) + 0x1000;
  if (val >= 0x47800000) {
    if ((fbits & 0x7fffffff) >= 0x47800000) {
      if (val < 0x7f800000) return int32ToBytes(sign | 0x7c00, a_twoByteBuffer);
      
      return int32ToBytes(sign | 0x7c00 | (fbits & 0x007fffff) >> 13, a_twoByteBuffer);
    }
    return int32ToBytes(sign | 0x7bff, a_twoByteBuffer);
  }
  if (val >= 0x38800000) return int32ToBytes(sign | val - 0x38000000 >> 13, a_twoByteBuffer);
  if (val < 0x33000000) return int32ToBytes(sign, a_twoByteBuffer);
  val = (fbits & 0x7fffffff) >> 23;
  return int32ToBytes(sign | ((fbits & 0x7fffff | 0x800000) + (0x800000 >> val - 102) >> 126 - val), a_twoByteBuffer);
}

void int32ToBytes(int32_t val, uint8_t* a_twoByteBuffer) {
  memcpy(a_twoByteBuffer, &val, 2);
}

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_SPI *sensor)
{
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}
