#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <math.h>

// =====================================================
// ASV SENSOR FUSION
// GPS NEO-7M + MPU6500 + QMC5883L
// ARDUINO UNO
//
// COMMUNICATION:
// USB SERIAL -> RASPBERRY PI
//
// INTERNAL RATE:
// MPU6500        200 Hz
// GYRO           200 Hz
// QMC            50 Hz
// FUSION         50 Hz
// USB OUTPUT     20 Hz
// =====================================================


// =====================================================
// GPS
// =====================================================

// GPS TX -> Arduino D4
// GPS RX -> Arduino D3

const int GPS_RX = 4;
const int GPS_TX = 3;

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

TinyGPSPlus gps;


// =====================================================
// MPU6500
// =====================================================

#define MPU_ADDR          0x68

#define MPU_WHO_AM_I      0x75
#define MPU_PWR_MGMT_1    0x6B
#define MPU_PWR_MGMT_2    0x6C
#define MPU_SMPLRT_DIV    0x19
#define MPU_CONFIG        0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C

#define MPU_ACCEL_XOUT_H  0x3B

#define MPU6500_WHO_AM_I  0x70


// =====================================================
// QMC5883L
// =====================================================

#define QMC_ADDR       0x0D
#define QMC_DATA_START 0x00
#define QMC_CONTROL    0x09
#define QMC_SET_RESET  0x0B


// =====================================================
// QMC CALIBRATION
// =====================================================

const float QMC_OFFSET_X = 1375.500;
const float QMC_OFFSET_Y = 1341.000;
const float QMC_OFFSET_Z = 1006.000;

const float QMC_SCALE_X = 0.920889;
const float QMC_SCALE_Y = 0.999217;
const float QMC_SCALE_Z = 1.094919;


// =====================================================
// HEADING OFFSET
// =====================================================

const float HEADING_OFFSET = 0.0;


// =====================================================
// SENSOR SCALE
// =====================================================

const float GYRO_SCALE  = 131.0;
const float ACCEL_SCALE = 16384.0;


// =====================================================
// RAW SENSOR
// =====================================================

int16_t axRaw = 0;
int16_t ayRaw = 0;
int16_t azRaw = 0;

int16_t gxRaw = 0;
int16_t gyRaw = 0;
int16_t gzRaw = 0;

int16_t magXRaw = 0;
int16_t magYRaw = 0;
int16_t magZRaw = 0;


// =====================================================
// GYRO BIAS
// =====================================================

float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;


// =====================================================
// ZERO REFERENCE
// =====================================================

float rollZero = 0.0;
float pitchZero = 0.0;


// =====================================================
// FINAL ROLL / PITCH
// =====================================================

float roll = 0.0;
float pitch = 0.0;


// =====================================================
// HEADING
// =====================================================

// Heading dari QMC
float magHeading = 0.0;

// Heading final fusion QMC + Gyro
float heading = 0.0;

bool magHeadingValid = false;
bool headingInitialized = false;


// =====================================================
// HEADING FUSION
// =====================================================

const float HEADING_ALPHA = 0.98;


// =====================================================
// QMC UPDATE
// =====================================================

// 20 ms = 50 Hz

const unsigned long MAG_UPDATE_INTERVAL = 20;

unsigned long lastMagUpdate = 0;


// =====================================================
// ROLL / PITCH FILTER
// =====================================================

const float GYRO_LPF_ALPHA = 0.85;
const float ACCEL_LPF_ALPHA = 0.30;
const float COMPLEMENTARY_ALPHA = 0.98;


// =====================================================
// GYRO DEAD BAND
// =====================================================

const float GYRO_DEADBAND = 0.08;


// =====================================================
// ZERO LOCK
// =====================================================

const float ZERO_LOCK_ANGLE = 0.12;
const float ZERO_LOCK_GYRO = 0.10;


// =====================================================
// FILTER MEMORY
// =====================================================

float gyroRollFiltered = 0.0;
float gyroPitchFiltered = 0.0;

float accelRollFiltered = 0.0;
float accelPitchFiltered = 0.0;


// =====================================================
// MEDIAN BUFFER
// =====================================================

float rollHistory[3] = {
  0.0,
  0.0,
  0.0
};

float pitchHistory[3] = {
  0.0,
  0.0,
  0.0
};

int historyIndex = 0;


// =====================================================
// FILTER STATUS
// =====================================================

bool filterInitialized = false;


// =====================================================
// EEPROM
// =====================================================

#define EEPROM_BIAS_X_ADDR  0
#define EEPROM_BIAS_Y_ADDR  4
#define EEPROM_BIAS_Z_ADDR  8

#define EEPROM_MARKER_ADDR  12
#define EEPROM_MARKER       0xAB


// =====================================================
// TIMING
// =====================================================

unsigned long lastIMUTime = 0;


// =====================================================
// USB NAVIGATION OUTPUT
// =====================================================

// 20 Hz = 50 ms

const unsigned long NAV_OUTPUT_INTERVAL = 50;

unsigned long lastNavOutput = 0;


// =====================================================
// PACKET SEQUENCE
// =====================================================

uint32_t packetSequence = 0;


// =====================================================
// GPS VALIDATION
// =====================================================

// Untuk navigasi awal.
// Nanti threshold bisa dituning berdasarkan pengujian laut.

const float GPS_MAX_HDOP = 3.0;


// =====================================================
// GPS COURSE VALIDATION
// =====================================================
//
// GPS course tidak dipercaya saat terlalu lambat.
//
// =====================================================

const float GPS_COURSE_MIN_SPEED_KMPH = 1.00;


// =====================================================
// INVALID FLOAT
// =====================================================

const float INVALID_FLOAT = -999.0;


// =====================================================
// MEDIAN 3
// =====================================================

float median3(
  float a,
  float b,
  float c
)
{
  if (a > b)
  {
    float temp = a;
    a = b;
    b = temp;
  }

  if (b > c)
  {
    float temp = b;
    b = c;
    c = temp;
  }

  if (a > b)
  {
    float temp = a;
    a = b;
    b = temp;
  }

  return b;
}


// =====================================================
// WRITE MPU REGISTER
// =====================================================

bool writeMPU(
  uint8_t reg,
  uint8_t data
)
{
  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(reg);
  Wire.write(data);

  return
    Wire.endTransmission() == 0;
}


// =====================================================
// READ MPU REGISTER
// =====================================================

bool readMPURegister(
  uint8_t reg,
  uint8_t &data
)
{
  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(reg);

  if (
    Wire.endTransmission(false) != 0
  )
  {
    return false;
  }


  uint8_t count =
    Wire.requestFrom(
      MPU_ADDR,
      (uint8_t)1,
      (uint8_t)true
    );


  if (count != 1)
  {
    return false;
  }


  data =
    Wire.read();

  return true;
}


// =====================================================
// READ MPU6500
// =====================================================

bool readMPU()
{
  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(
    MPU_ACCEL_XOUT_H
  );


  if (
    Wire.endTransmission(false) != 0
  )
  {
    return false;
  }


  uint8_t count =
    Wire.requestFrom(
      MPU_ADDR,
      (uint8_t)14,
      (uint8_t)true
    );


  if (count != 14)
  {
    return false;
  }


  // ===================================================
  // ACC X
  // ===================================================

  axRaw =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  // ===================================================
  // ACC Y
  // ===================================================

  ayRaw =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  // ===================================================
  // ACC Z
  // ===================================================

  azRaw =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  // ===================================================
  // TEMP
  // ===================================================

  Wire.read();
  Wire.read();


  // ===================================================
  // GYRO X
  // ===================================================

  gxRaw =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  // ===================================================
  // GYRO Y
  // ===================================================

  gyRaw =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  // ===================================================
  // GYRO Z
  // ===================================================

  gzRaw =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  return true;
}


// =====================================================
// INITIALIZE MPU6500
// =====================================================

bool initMPU6500()
{
  Wire.beginTransmission(
    MPU_ADDR
  );


  if (
    Wire.endTransmission() != 0
  )
  {
    return false;
  }


  // ===================================================
  // WHO AM I
  // ===================================================

  uint8_t whoAmI = 0;


  if (
    !readMPURegister(
      MPU_WHO_AM_I,
      whoAmI
    )
  )
  {
    return false;
  }


  if (
    whoAmI != MPU6500_WHO_AM_I
  )
  {
    return false;
  }


  // ===================================================
  // RESET
  // ===================================================

  writeMPU(
    MPU_PWR_MGMT_1,
    0x80
  );

  delay(100);


  // ===================================================
  // WAKE
  // ===================================================

  writeMPU(
    MPU_PWR_MGMT_1,
    0x01
  );

  writeMPU(
    MPU_PWR_MGMT_2,
    0x00
  );

  delay(100);


  // ===================================================
  // SAMPLE RATE
  // ===================================================
  //
  // 1 kHz / (1 + 4) = 200 Hz
  //
  // =====================================================

  writeMPU(
    MPU_SMPLRT_DIV,
    4
  );


  // ===================================================
  // DLPF
  // ===================================================

  writeMPU(
    MPU_CONFIG,
    0x03
  );


  // ===================================================
  // ACCEL ±2G
  // ===================================================

  writeMPU(
    MPU_ACCEL_CONFIG,
    0x00
  );


  // ===================================================
  // GYRO ±250 DEG/S
  // ===================================================

  writeMPU(
    MPU_GYRO_CONFIG,
    0x00
  );


  delay(100);

  return true;
}


// =====================================================
// WRITE QMC
// =====================================================

bool writeQMC(
  uint8_t reg,
  uint8_t data
)
{
  Wire.beginTransmission(
    QMC_ADDR
  );

  Wire.write(reg);
  Wire.write(data);

  return
    Wire.endTransmission() == 0;
}


// =====================================================
// READ QMC5883L
// =====================================================

bool readQMC()
{
  Wire.beginTransmission(
    QMC_ADDR
  );

  Wire.write(
    QMC_DATA_START
  );


  if (
    Wire.endTransmission() != 0
  )
  {
    return false;
  }


  Wire.requestFrom(
    QMC_ADDR,
    (uint8_t)6
  );


  if (
    Wire.available() == 6
  )
  {
    uint8_t xl =
      Wire.read();

    uint8_t xh =
      Wire.read();

    uint8_t yl =
      Wire.read();

    uint8_t yh =
      Wire.read();

    uint8_t zl =
      Wire.read();

    uint8_t zh =
      Wire.read();


    magXRaw =
      (int16_t)(
        ((uint16_t)xh << 8) |
        xl
      );


    magYRaw =
      (int16_t)(
        ((uint16_t)yh << 8) |
        yl
      );


    magZRaw =
      (int16_t)(
        ((uint16_t)zh << 8) |
        zl
      );


    return true;
  }


  return false;
}


// =====================================================
// INITIALIZE QMC5883L
// =====================================================
//
// 0x1D
//
// OSR  = 512
// RNG  = 8G
// ODR  = 200Hz
// MODE = Continuous
//
// =====================================================

bool initQMC()
{
  Wire.beginTransmission(
    QMC_ADDR
  );


  if (
    Wire.endTransmission() != 0
  )
  {
    return false;
  }


  // ===================================================
  // SOFT RESET
  // ===================================================

  if (
    !writeQMC(
      QMC_SET_RESET,
      0x01
    )
  )
  {
    return false;
  }


  delay(20);


  // ===================================================
  // CONTROL
  // ===================================================

  if (
    !writeQMC(
      QMC_CONTROL,
      0x1D
    )
  )
  {
    return false;
  }


  delay(500);

  return true;
}


// =====================================================
// CHECK GYRO CALIBRATION
// =====================================================

bool calibrationExists()
{
  return
    EEPROM.read(
      EEPROM_MARKER_ADDR
    )
    ==
    EEPROM_MARKER;
}


// =====================================================
// LOAD GYRO CALIBRATION
// =====================================================

void loadCalibration()
{
  EEPROM.get(
    EEPROM_BIAS_X_ADDR,
    gyroBiasX
  );

  EEPROM.get(
    EEPROM_BIAS_Y_ADDR,
    gyroBiasY
  );

  EEPROM.get(
    EEPROM_BIAS_Z_ADDR,
    gyroBiasZ
  );


  Serial.println(
    "ASV,INFO,GYRO_CALIBRATION=EEPROM"
  );


  Serial.print(
    "ASV,INFO,BIAS_X="
  );

  Serial.println(
    gyroBiasX,
    5
  );


  Serial.print(
    "ASV,INFO,BIAS_Y="
  );

  Serial.println(
    gyroBiasY,
    5
  );


  Serial.print(
    "ASV,INFO,BIAS_Z="
  );

  Serial.println(
    gyroBiasZ,
    5
  );
}


// =====================================================
// SAVE GYRO CALIBRATION
// =====================================================

void saveCalibration()
{
  EEPROM.put(
    EEPROM_BIAS_X_ADDR,
    gyroBiasX
  );

  EEPROM.put(
    EEPROM_BIAS_Y_ADDR,
    gyroBiasY
  );

  EEPROM.put(
    EEPROM_BIAS_Z_ADDR,
    gyroBiasZ
  );


  EEPROM.update(
    EEPROM_MARKER_ADDR,
    EEPROM_MARKER
  );
}


// =====================================================
// CALIBRATE GYRO
// =====================================================
//
// HANYA DIPANGGIL DENGAN COMMAND K.
//
// =====================================================

void calibrateGyro()
{
  Serial.println(
    "ASV,INFO,GYRO_CALIBRATION_START"
  );


  Serial.println(
    "ASV,INFO,KEEP_SENSOR_STILL"
  );


  delay(3000);


  const int samples = 2000;


  long sumGX = 0;
  long sumGY = 0;
  long sumGZ = 0;


  int validSamples = 0;


  for (
    int i = 0;
    i < samples;
    i++
  )
  {
    if (
      readMPU()
    )
    {
      sumGX += gxRaw;
      sumGY += gyRaw;
      sumGZ += gzRaw;

      validSamples++;
    }


    delay(2);
  }


  if (
    validSamples == 0
  )
  {
    Serial.println(
      "ASV,ERROR,GYRO_CALIBRATION"
    );

    return;
  }


  float avgGX =
    (float)sumGX /
    validSamples;


  float avgGY =
    (float)sumGY /
    validSamples;


  float avgGZ =
    (float)sumGZ /
    validSamples;


  gyroBiasX =
    avgGX /
    GYRO_SCALE;


  gyroBiasY =
    avgGY /
    GYRO_SCALE;


  gyroBiasZ =
    avgGZ /
    GYRO_SCALE;


  saveCalibration();


  Serial.println(
    "ASV,INFO,GYRO_CALIBRATION_DONE"
  );


  Serial.print(
    "ASV,BIAS,X="
  );

  Serial.println(
    gyroBiasX,
    5
  );


  Serial.print(
    "ASV,BIAS,Y="
  );

  Serial.println(
    gyroBiasY,
    5
  );


  Serial.print(
    "ASV,BIAS,Z="
  );

  Serial.println(
    gyroBiasZ,
    5
  );
}


// =====================================================
// CALCULATE ACCEL ANGLES
// =====================================================

void calculateAccelAngles(
  float &rollAccel,
  float &pitchAccel
)
{
  float ax =
    (float)axRaw /
    ACCEL_SCALE;


  float ay =
    (float)ayRaw /
    ACCEL_SCALE;


  float az =
    (float)azRaw /
    ACCEL_SCALE;


  rollAccel =
    atan2(
      ax,
      sqrt(
        ay * ay +
        az * az
      )
    )
    *
    180.0 /
    PI;


  pitchAccel =
    atan2(
      -ay,
      sqrt(
        ax * ax +
        az * az
      )
    )
    *
    180.0 /
    PI;
}


// =====================================================
// CALIBRATE ROLL / PITCH ZERO
// =====================================================

void calibrateAngleZero()
{
  Serial.println(
    "ASV,INFO,ANGLE_ZERO_START"
  );


  Serial.println(
    "ASV,INFO,POSITION_ASV_NORMAL"
  );


  delay(3000);


  const int samples = 500;


  float sumRoll = 0.0;
  float sumPitch = 0.0;


  int validSamples = 0;


  for (
    int i = 0;
    i < samples;
    i++
  )
  {
    if (
      readMPU()
    )
    {
      float r;
      float p;


      calculateAccelAngles(
        r,
        p
      );


      sumRoll += r;
      sumPitch += p;


      validSamples++;
    }


    delay(4);
  }


  if (
    validSamples == 0
  )
  {
    Serial.println(
      "ASV,ERROR,ANGLE_ZERO"
    );

    return;
  }


  rollZero =
    sumRoll /
    validSamples;


  pitchZero =
    sumPitch /
    validSamples;


  Serial.print(
    "ASV,INFO,ROLL_ZERO="
  );

  Serial.println(
    rollZero,
    3
  );


  Serial.print(
    "ASV,INFO,PITCH_ZERO="
  );

  Serial.println(
    pitchZero,
    3
  );
}


// =====================================================
// UPDATE ROLL / PITCH
// 200 Hz INTERNAL
//
// KOREKSI:
// - Gyro X/Y tidak langsung dianggap Euler rate.
// - Gyro Z ikut diperhitungkan ketika sensor miring.
// - Memperhitungkan coupling antar sumbu.
// - Accelerometer tetap menjadi referensi gravitasi.
// - Complementary filter tetap digunakan.
// =====================================================

bool updateOrientation(
  float &dtOut
)
{
  // ===================================================
  // HITUNG DT
  // ===================================================

  unsigned long now =
    micros();

  float dt =
    (
      now -
      lastIMUTime
    )
    /
    1000000.0;


  lastIMUTime =
    now;


  if (
    dt <= 0.0 ||
    dt > 0.05
  )
  {
    return false;
  }


  // ===================================================
  // BACA MPU6500
  // ===================================================

  if (
    !readMPU()
  )
  {
    return false;
  }


  dtOut =
    dt;


  // ===================================================
  // GYROSCOPE
  // ===================================================

  float gx =
    (
      (float)gxRaw /
      GYRO_SCALE
    )
    -
    gyroBiasX;


  float gy =
    (
      (float)gyRaw /
      GYRO_SCALE
    )
    -
    gyroBiasY;


  float gz =
    (
      (float)gzRaw /
      GYRO_SCALE
    )
    -
    gyroBiasZ;


  // ===================================================
  // GYRO DEAD BAND
  // ===================================================

  if (
    fabs(gx) <
    GYRO_DEADBAND
  )
  {
    gx =
      0.0;
  }


  if (
    fabs(gy) <
    GYRO_DEADBAND
  )
  {
    gy =
      0.0;
  }


  if (
    fabs(gz) <
    GYRO_DEADBAND
  )
  {
    gz =
      0.0;
  }


  // ===================================================
  // ACCELEROMETER ANGLE
  // ===================================================

  float rollAccel;
  float pitchAccel;


  calculateAccelAngles(
    rollAccel,
    pitchAccel
  );


  // ===================================================
  // ZERO REFERENCE
  // ===================================================

  rollAccel -=
    rollZero;


  pitchAccel -=
    pitchZero;


  // ===================================================
  // MEDIAN FILTER
  // ===================================================

  rollHistory[historyIndex] =
    rollAccel;


  pitchHistory[historyIndex] =
    pitchAccel;


  historyIndex++;


  if (
    historyIndex >= 3
  )
  {
    historyIndex =
      0;
  }


  float rollMedian =
    median3(
      rollHistory[0],
      rollHistory[1],
      rollHistory[2]
    );


  float pitchMedian =
    median3(
      pitchHistory[0],
      pitchHistory[1],
      pitchHistory[2]
    );


  // ===================================================
  // FILTER INITIALIZATION
  // ===================================================

  if (
    !filterInitialized
  )
  {
    gyroRollFiltered =
      gx;


    gyroPitchFiltered =
      gy;


    accelRollFiltered =
      rollMedian;


    accelPitchFiltered =
      pitchMedian;


    roll =
      rollMedian;


    pitch =
      pitchMedian;


    filterInitialized =
      true;


    return true;
  }


  // ===================================================
  // GYRO LOW PASS FILTER
  // ===================================================

  gyroRollFiltered =
    (
      GYRO_LPF_ALPHA *
      gx
    )
    +
    (
      (
        1.0 -
        GYRO_LPF_ALPHA
      )
      *
      gyroRollFiltered
    );


  gyroPitchFiltered =
    (
      GYRO_LPF_ALPHA *
      gy
    )
    +
    (
      (
        1.0 -
        GYRO_LPF_ALPHA
      )
      *
      gyroPitchFiltered
    );


  // ===================================================
  // ACCELEROMETER LOW PASS FILTER
  // ===================================================

  accelRollFiltered =
    (
      ACCEL_LPF_ALPHA *
      rollMedian
    )
    +
    (
      (
        1.0 -
        ACCEL_LPF_ALPHA
      )
      *
      accelRollFiltered
    );


  accelPitchFiltered =
    (
      ACCEL_LPF_ALPHA *
      pitchMedian
    )
    +
    (
      (
        1.0 -
        ACCEL_LPF_ALPHA
      )
      *
      accelPitchFiltered
    );


  // ===================================================
  // KONVERSI ROLL / PITCH KE RADIAN
  // ===================================================

  float rollRad =
    roll *
    PI /
    180.0;


  float pitchRad =
    pitch *
    PI /
    180.0;


  // ===================================================
  // PROTEKSI TAN(PITCH)
  // ===================================================

  float pitchSafe =
    pitchRad;


  // Batasi sebelum mendekati ±90°
  // untuk menghindari tan() terlalu besar.

  if (
    pitchSafe >
    1.48353
  )
  {
    pitchSafe =
      1.48353;
  }


  if (
    pitchSafe <
    -1.48353
  )
  {
    pitchSafe =
      -1.48353;
  }


  // ===================================================
  // EULER RATE
  //
  // Body angular velocity:
  //
  //     gx
  //     gy
  //     gz
  //
  // diubah menjadi Euler roll/pitch rate.
  // ===================================================

  float sinRoll =
    sin(
      rollRad
    );


  float cosRoll =
    cos(
      rollRad
    );


  float tanPitch =
    tan(
      pitchSafe
    );


  // ===================================================
  // ROLL RATE
  // ===================================================

  float rollDot =
    gyroRollFiltered
    +
    (
      sinRoll *
      tanPitch *
      gyroPitchFiltered
    )
    +
    (
      cosRoll *
      tanPitch *
      gz
    );


  // ===================================================
  // PITCH RATE
  // ===================================================

  float pitchDot =
    (
      cosRoll *
      gyroPitchFiltered
    )
    -
    (
      sinRoll *
      gz
    );


  // ===================================================
  // INTEGRASI GYRO
  // ===================================================

  float rollGyro =
    roll
    +
    (
      rollDot *
      dt
    );


  float pitchGyro =
    pitch
    +
    (
      pitchDot *
      dt
    );


  // ===================================================
  // NORMALISASI ROLL
  // ===================================================

  while (
    rollGyro >
    180.0
  )
  {
    rollGyro -=
      360.0;
  }


  while (
    rollGyro <
    -180.0
  )
  {
    rollGyro +=
      360.0;
  }


  // ===================================================
  // NORMALISASI PITCH
  // ===================================================

  while (
    pitchGyro >
    180.0
  )
  {
    pitchGyro -=
      360.0;
  }


  while (
    pitchGyro <
    -180.0
  )
  {
    pitchGyro +=
      360.0;
  }


  // ===================================================
  // COMPLEMENTARY FILTER
  //
  // GYRO  = respons gerakan cepat
  // ACCEL = referensi gravitasi jangka panjang
  // ===================================================

  roll =
    (
      COMPLEMENTARY_ALPHA *
      rollGyro
    )
    +
    (
      (
        1.0 -
        COMPLEMENTARY_ALPHA
      )
      *
      accelRollFiltered
    );


  pitch =
    (
      COMPLEMENTARY_ALPHA *
      pitchGyro
    )
    +
    (
      (
        1.0 -
        COMPLEMENTARY_ALPHA
      )
      *
      accelPitchFiltered
    );


  // ===================================================
  // ZERO LOCK
  // ===================================================

  if (
    fabs(roll) <
    ZERO_LOCK_ANGLE

    &&

    fabs(pitch) <
    ZERO_LOCK_ANGLE

    &&

    fabs(gyroRollFiltered) <
    ZERO_LOCK_GYRO

    &&

    fabs(gyroPitchFiltered) <
    ZERO_LOCK_GYRO
  )
  {
    roll =
      0.0;


    pitch =
      0.0;
  }


  return true;
}


// =====================================================
// WRAP -180 ... +180
// =====================================================

float wrap180(
  float angle
)
{
  while (
    angle >
    180.0
  )
  {
    angle -=
      360.0;
  }


  while (
    angle <
    -180.0
  )
  {
    angle +=
      360.0;
  }


  return angle;
}


// =====================================================
// NORMALIZE 0 ... 360
// =====================================================

float normalize360(
  float angle
)
{
  while (
    angle >=
    360.0
  )
  {
    angle -=
      360.0;
  }


  while (
    angle <
    0.0
  )
  {
    angle +=
      360.0;
  }


  return angle;
}


// =====================================================
// CALCULATE MAGNETIC HEADING
// QMC METODE AWAL
// =====================================================
//
// calX = (rawX - OFFSET_X) * SCALE_X
// calY = (rawY - OFFSET_Y) * SCALE_Y
//
// heading = atan2(calY, calX)
//
// TANPA TILT COMPENSATION
//
// =====================================================

bool calculateMagneticHeading(
  float &result
)
{
  if (
    !readQMC()
  )
  {
    return false;
  }


  // ===================================================
  // CAL X
  // ===================================================

  float calX =
    (
      (float)magXRaw -
      QMC_OFFSET_X
    )
    *
    QMC_SCALE_X;


  // ===================================================
  // CAL Y
  // ===================================================

  float calY =
    (
      (float)magYRaw -
      QMC_OFFSET_Y
    )
    *
    QMC_SCALE_Y;


  // ===================================================
  // CAL Z
  // ===================================================

  float calZ =
    (
      (float)magZRaw -
      QMC_OFFSET_Z
    )
    *
    QMC_SCALE_Z;


  (void)calZ;


  // ===================================================
  // CHECK
  // ===================================================

  if (
    fabs(calX) < 0.001 &&
    fabs(calY) < 0.001
  )
  {
    return false;
  }


  // ===================================================
  // HEADING
  // ===================================================

  float hdg =
    atan2(
      calY,
      calX
    )
    *
    180.0 /
    PI;


  // ===================================================
  // NORMALIZE
  // ===================================================

  if (
    hdg <
    0.0
  )
  {
    hdg +=
      360.0;
  }


  hdg +=
    HEADING_OFFSET;


  hdg =
    normalize360(
      hdg
    );


  result =
    hdg;


  return true;
}


// =====================================================
// UPDATE HEADING FUSION
// =====================================================
//
// GYRO      = 200 Hz
// QMC CORR. = 50 Hz
//
// =====================================================

void updateHeading(
  float dt
)
{
  // ===================================================
  // GYRO Z
  // ===================================================

  float gz =
    (
      (float)gzRaw /
      GYRO_SCALE
    )
    -
    gyroBiasZ;


  // ===================================================
  // DEAD BAND
  // ===================================================

  if (
    fabs(gz) <
    GYRO_DEADBAND
  )
  {
    gz =
      0.0;
  }


  // ===================================================
  // GYRO INTEGRATION
  // ===================================================

  if (
    headingInitialized
  )
  {
    heading +=
      gz *
      dt;


    heading =
      normalize360(
        heading
      );
  }


  // ===================================================
  // QMC CORRECTION
  // 50 Hz
  // ===================================================

  unsigned long now =
    millis();


  if (
    now -
    lastMagUpdate
    >=
    MAG_UPDATE_INTERVAL
  )
  {
    lastMagUpdate =
      now;


    float newMagHeading =
      0.0;


    if (
      calculateMagneticHeading(
        newMagHeading
      )
    )
    {
      magHeading =
        newMagHeading;


      magHeadingValid =
        true;


      // =================================================
      // FIRST INITIALIZATION
      // =================================================

      if (
        !headingInitialized
      )
      {
        heading =
          magHeading;


        headingInitialized =
          true;
      }


      // =================================================
      // FUSION
      // =================================================

      else
      {
        float error =
          wrap180(
            magHeading -
            heading
          );


        heading +=
          (
            1.0 -
            HEADING_ALPHA
          )
          *
          error;


        heading =
          normalize360(
            heading
          );
      }
    }
  }
}


// =====================================================
// RESET FILTER
// =====================================================

void resetFilters()
{
  gyroRollFiltered = 0.0;
  gyroPitchFiltered = 0.0;

  accelRollFiltered = 0.0;
  accelPitchFiltered = 0.0;


  roll = 0.0;
  pitch = 0.0;


  rollHistory[0] = 0.0;
  rollHistory[1] = 0.0;
  rollHistory[2] = 0.0;


  pitchHistory[0] = 0.0;
  pitchHistory[1] = 0.0;
  pitchHistory[2] = 0.0;


  historyIndex = 0;


  filterInitialized =
    false;


  heading = 0.0;
  magHeading = 0.0;


  magHeadingValid =
    false;


  headingInitialized =
    false;


  lastMagUpdate =
    millis();


  lastIMUTime =
    micros();
}


// =====================================================
// GPS VALIDATION
// =====================================================

bool isGPSValid()
{
  if (
    !gps.location.isValid()
  )
  {
    return false;
  }


  if (
    !gps.hdop.isValid()
  )
  {
    return false;
  }


  if (
    gps.hdop.hdop() >
    GPS_MAX_HDOP
  )
  {
    return false;
  }


  return true;
}


// =====================================================
// GPS COURSE VALIDATION
// =====================================================

bool isGPSCourseValid()
{
  if (
    !isGPSValid()
  )
  {
    return false;
  }


  if (
    !gps.speed.isValid()
  )
  {
    return false;
  }


  if (
    !gps.course.isValid()
  )
  {
    return false;
  }


  if (
    gps.speed.kmph() <
    GPS_COURSE_MIN_SPEED_KMPH
  )
  {
    return false;
  }


  return true;
}


// =====================================================
// HEADING VALIDATION
// =====================================================

bool isHeadingValid()
{
  return
    headingInitialized &&
    magHeadingValid;
}


// =====================================================
// HEADING ERROR
// =====================================================

float getHeadingError()
{
  if (
    !isHeadingValid()
  )
  {
    return INVALID_FLOAT;
  }


  return wrap180(
    magHeading -
    heading
  );
}


// =====================================================
// SENSOR STATUS
// =====================================================
//
// 2 = OK
// 1 = DEGRADED
// 0 = INVALID
//
// =====================================================

uint8_t getSensorStatus()
{
  bool gpsValid =
    isGPSValid();


  bool headingValid =
    isHeadingValid();


  if (
    gpsValid &&
    headingValid
  )
  {
    return 2;
  }


  if (
    gpsValid ||
    headingValid
  )
  {
    return 1;
  }


  return 0;
}


// =====================================================
// PRINT FLOAT
// =====================================================

void printFloat(
  float value,
  uint8_t digits
)
{
  Serial.print(
    value,
    digits
  );
}


// =====================================================
// SEND USB NAVIGATION PACKET
// =====================================================
//
// 20 Hz
//
// Format:
//
// ASV,
// SEQ=,
// T=,
// GPS=,
// SAT=,
// HDOP=,
// LAT=,
// LON=,
// ALT=,
// SPD=,
// CRS=,
// CRSV=,
// ROLL=,
// PITCH=,
// MAG=,
// HDG=,
// HDGV=,
// HERR=,
// STS=
//
// =====================================================

void sendNavigationPacket()
{
  unsigned long now =
    millis();


  if (
    now -
    lastNavOutput
    <
    NAV_OUTPUT_INTERVAL
  )
  {
    return;
  }


  lastNavOutput =
    now;


  // ===================================================
  // VALIDATION
  // ===================================================

  bool gpsValid =
    isGPSValid();


  bool courseValid =
    isGPSCourseValid();


  bool headingValid =
    isHeadingValid();


  uint8_t sensorStatus =
    getSensorStatus();


  float headingError =
    getHeadingError();


  // ===================================================
  // START PACKET
  // ===================================================

  Serial.print(
    "ASV"
  );


  Serial.print(
    ",SEQ="
  );

  Serial.print(
    packetSequence
  );


  Serial.print(
    ",T="
  );

  Serial.print(
    now
  );


  // ===================================================
  // GPS VALID
  // ===================================================

  Serial.print(
    ",GPS="
  );

  Serial.print(
    gpsValid ? 1 : 0
  );


  // ===================================================
  // SATELLITES
  // ===================================================

  Serial.print(
    ",SAT="
  );

  if (
    gps.satellites.isValid()
  )
  {
    Serial.print(
      gps.satellites.value()
    );
  }
  else
  {
    Serial.print(
      -1
    );
  }


  // ===================================================
  // HDOP
  // ===================================================

  Serial.print(
    ",HDOP="
  );

  if (
    gps.hdop.isValid()
  )
  {
    printFloat(
      gps.hdop.hdop(),
      2
    );
  }
  else
  {
    printFloat(
      INVALID_FLOAT,
      2
    );
  }


  // ===================================================
  // LATITUDE
  // ===================================================

  Serial.print(
    ",LAT="
  );

  if (
    gps.location.isValid()
  )
  {
    printFloat(
      gps.location.lat(),
      6
    );
  }
  else
  {
    printFloat(
      INVALID_FLOAT,
      6
    );
  }


  // ===================================================
  // LONGITUDE
  // ===================================================

  Serial.print(
    ",LON="
  );

  if (
    gps.location.isValid()
  )
  {
    printFloat(
      gps.location.lng(),
      6
    );
  }
  else
  {
    printFloat(
      INVALID_FLOAT,
      6
    );
  }


  // ===================================================
  // ALTITUDE
  // ===================================================

  Serial.print(
    ",ALT="
  );

  if (
    gps.altitude.isValid()
  )
  {
    printFloat(
      gps.altitude.meters(),
      2
    );
  }
  else
  {
    printFloat(
      INVALID_FLOAT,
      2
    );
  }


  // ===================================================
  // SPEED / SOG
  // ===================================================

  Serial.print(
    ",SOG="
  );

  if (
    gps.speed.isValid()
  )
  {
    printFloat(
      gps.speed.kmph(),
      2
    );
  }
  else
  {
    printFloat(
      INVALID_FLOAT,
      2
    );
  }


  // ===================================================
  // GPS COURSE / COG
  // ===================================================

  Serial.print(
    ",COG="
  );

  if (
    gps.course.isValid()
  )
  {
    printFloat(
      gps.course.deg(),
      2
    );
  }
  else
  {
    printFloat(
      INVALID_FLOAT,
      2
    );
  }


  // ===================================================
  // GPS COURSE VALID
  // ===================================================

  Serial.print(
    ",CRSV="
  );

  Serial.print(
    courseValid ? 1 : 0
  );


  // ===================================================
  // ROLL
  // ===================================================

  Serial.print(
    ",ROLL="
  );

  printFloat(
    roll,
    2
  );


  // ===================================================
  // PITCH
  // ===================================================

  Serial.print(
    ",PITCH="
  );

  printFloat(
    pitch,
    2
  );


  // ===================================================
  // MAG HEADING
  // ===================================================

  Serial.print(
    ",MAG="
  );

  if (
    magHeadingValid
  )
  {
    printFloat(
      magHeading,
      2
    );
  }
  else
  {
    printFloat(
      INVALID_FLOAT,
      2
    );
  }


  // ===================================================
  // FUSED HEADING
  // ===================================================

  Serial.print(
    ",HDG="
  );

  if (
    headingValid
  )
  {
    printFloat(
      heading,
      2
    );
  }
  else
  {
    printFloat(
      INVALID_FLOAT,
      2
    );
  }


  // ===================================================
  // HEADING VALID
  // ===================================================

  Serial.print(
    ",HDGV="
  );

  Serial.print(
    headingValid ? 1 : 0
  );


  // ===================================================
  // HEADING ERROR
  // ===================================================

  Serial.print(
    ",HERR="
  );

  printFloat(
    headingError,
    2
  );


  // ===================================================
  // SENSOR STATUS
  // ===================================================

  Serial.print(
    ",STS="
  );

  Serial.print(
    sensorStatus
  );


  // ===================================================
  // END PACKET
  // ===================================================

  Serial.println();


  // ===================================================
  // NEXT SEQUENCE
  // ===================================================

  packetSequence++;
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  // ===================================================
  // USB SERIAL
  // ===================================================

  Serial.begin(
    115200
  );


  // ===================================================
  // I2C
  // ===================================================

  Wire.begin();

  Wire.setClock(
    400000
  );

  delay(300);


  // ===================================================
  // GPS
  // ===================================================

  gpsSerial.begin(
    9600
  );


  // ===================================================
  // MPU6500
  // ===================================================

  if (
    !initMPU6500()
  )
  {
    Serial.println(
      "ASV,ERROR,MPU6500"
    );


    while (1)
    {
      delay(1000);
    }
  }


  Serial.println(
    "ASV,READY,MPU6500"
  );


  // ===================================================
  // QMC5883L
  // ===================================================

  if (
    !initQMC()
  )
  {
    Serial.println(
      "ASV,ERROR,QMC5883L"
    );


    while (1)
    {
      delay(1000);
    }
  }


  Serial.println(
    "ASV,READY,QMC5883L"
  );


  // ===================================================
  // GYRO CALIBRATION
  // ===================================================
  //
  // TIDAK OTOMATIS.
  //
  // ===================================================

  if (
    calibrationExists()
  )
  {
    loadCalibration();
  }
  else
  {
    gyroBiasX = 0.0;
    gyroBiasY = 0.0;
    gyroBiasZ = 0.0;


    Serial.println(
      "ASV,WARNING,GYRO_NOT_CALIBRATED"
    );


    Serial.println(
      "ASV,INFO,SEND_K_TO_CALIBRATE"
    );
  }


  // ===================================================
  // ZERO ROLL / PITCH
  // ===================================================

  calibrateAngleZero();


  // ===================================================
  // RESET FILTER
  // ===================================================

  resetFilters();


  // ===================================================
  // OUTPUT TIMER
  // ===================================================

  lastNavOutput =
    millis();


  // ===================================================
  // READY
  // ===================================================

  Serial.println(
    "ASV,READY,USB,115200"
  );
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // ===================================================
  // GPS RECEIVE
  // ===================================================
  //
  // Baca terus supaya buffer GPS tidak tertinggal.
  //
  // ===================================================

  while (
    gpsSerial.available() > 0
  )
  {
    char c =
      gpsSerial.read();


    gps.encode(c);
  }


  // ===================================================
  // USB COMMAND
  // ===================================================
  //
  // K / k = KALIBRASI GYRO
  //
  // ===================================================

  if (
    Serial.available() > 0
  )
  {
    char command =
      Serial.read();


    if (
      command == 'K' ||
      command == 'k'
    )
    {
      Serial.println(
        "ASV,CMD,K"
      );


      // -----------------------------------------------
      // KALIBRASI GYRO
      // -----------------------------------------------

      calibrateGyro();


      // -----------------------------------------------
      // RESET FILTER
      // -----------------------------------------------

      resetFilters();


      // -----------------------------------------------
      // TIMER OUTPUT
      // -----------------------------------------------

      lastNavOutput =
        millis();


      Serial.println(
        "ASV,READY,GYRO"
      );
    }
  }


  // ===================================================
  // IMU UPDATE
  // ===================================================
  //
  // 200 Hz
  //
  // ===================================================

  static unsigned long lastIMUUpdate = 0;


  unsigned long now =
    micros();


  if (
    now -
    lastIMUUpdate
    >=
    5000
  )
  {
    lastIMUUpdate =
      now;


    float dt =
      0.0;


    if (
      updateOrientation(
        dt
      )
    )
    {
      updateHeading(
        dt
      );
    }
  }


  // ===================================================
  // USB NAVIGATION PACKET
  // ===================================================
  //
  // 20 Hz
  //
  // ===================================================

  sendNavigationPacket();
}