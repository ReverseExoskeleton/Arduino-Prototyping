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
#include "Fusion.h"

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.
#define CS_PIN_1 10   // Which pin you connect CS to for Sensor 1

ICM_20948_SPI myICM1; // Create an ICM_20948_SPI object

float samplePeriod = 0.01f; // seconds == 10 ms

FusionVector3 gyroscopeSensitivity = {1/65.5f,  1/65.5f,  1/65.5f};
FusionVector3 accelerometerSensitivity = {1/8192.0f, 1/8192.0f, 1/8192.0f};
FusionVector3 hardIronBias = {0.0f,0.0f,0.0f};
float _StationaryThreshold = 0.5f; // dps
float _Gain = 5.0f; 
float _MinMagField = 20.0f; // uT
float _MaxMagField = 70.0f; // uT
FusionBias fusionBias;
FusionAhrs fusionAhrs;

void setup()
{

  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };

  SPI_PORT.begin();

  myICM1.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

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
  delay(1000);
  
  // Initialise gyroscope bias correction algorithm
  FusionBiasInitialise(&fusionBias, _StationaryThreshold, samplePeriod);
  // Initialise AHRS algorithm
  FusionAhrsInitialise(&fusionAhrs, _Gain);
  // Set optional magnetic field limits
  FusionAhrsSetMagneticField(&fusionAhrs, _MinMagField, _MaxMagField);

  delay(1000);
  SERIAL_PORT.print("Data Start");
  delay(10);

}

void loop()
{
  if (myICM1.dataReady()) {
    myICM1.getAGMT(); // The values are only updated when you call 'getAGMT'
    SERIAL_PORT.print("Scaled: ");
    printScaledAGMT(&myICM1);
    SERIAL_PORT.print("\n");
    printQuaternion(&myICM1);
    SERIAL_PORT.print("\n\n");
    delay(10);
  } else {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }
}

void printQuaternion(ICM_20948_SPI* sensor) {
  // Calibrate gyroscope
  FusionVector3 uncalibratedGyroscope = {sensor->gyrX(), sensor->gyrY(), sensor->gyrZ()};
  FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY(), gyroscopeSensitivity, FUSION_VECTOR3_ZERO());

  // Calibrate accelerometer
  FusionVector3 uncalibratedAccelerometer = {sensor->accX() / 1000, sensor->accY() / 1000, sensor->accZ() / 1000};
  FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY(), accelerometerSensitivity, FUSION_VECTOR3_ZERO());

  // Calibrate magnetometer
  FusionVector3 uncalibratedMagnetometer = {sensor->magX(), sensor->magY(), sensor->magZ()};
  FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY(), hardIronBias);

  // Update gyroscope bias correction algorithm
  calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

  // Update AHRS algorithm
  FusionAhrsUpdate(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer, samplePeriod);

  // Print Euler angles
  FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));
  printf("Roll = %0.1f, Pitch = %0.1f, Yaw = %0.1f\r\n", eulerAngles.roll, eulerAngles.pitch, eulerAngles.yaw);

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
