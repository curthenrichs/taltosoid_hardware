#include <SparkFunMPU9250-DMP.h>
#include <ArduinoJson.h>
#include <stdint.h>

// {D8, D3, D4} = x in bootmode (x,y)'
// D8 = low, D4 = high
// D3 = low if UART, high if flash

#define FLEX_INDEX_PIN (A0)

typedef struct Quaternion {
  float x;
  float y;
  float z;
  float w;
} Quaternion_t;

typedef struct Vector3 {
  float x;
  float y;
  float z;
} Vector3_t;

typedef struct Sensors {
  Quaternion_t orientation_quaternion;
  Vector3 orientation_euler;
  float orientation_timestamp;
  
  Vector3 accel_raw;
  Vector3 gyro_raw;
  Vector3 magentic_raw;
  float raw_timestamp;
  
  float flex_0;
} Sensors_t;

static StaticJsonDocument<200> root;
static MPU9250_DMP imu;
static Sensors_t sensorData;

void setup() {
  Serial.begin(74880);
  imu.begin();
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL,10);

  sensorData.orientation_quaternion.x = 0;
  sensorData.orientation_quaternion.y = 0;
  sensorData.orientation_quaternion.z = 0;
  sensorData.orientation_quaternion.w = 0;
  sensorData.orientation_euler.x = 0;
  sensorData.orientation_euler.y = 0;
  sensorData.orientation_euler.z = 0;
  sensorData.orientation_timestamp = 0;
  sensorData.accel_raw.x = 0;
  sensorData.accel_raw.y = 0;
  sensorData.accel_raw.z = 0;
  sensorData.gyro_raw.x = 0;
  sensorData.gyro_raw.y = 0;
  sensorData.gyro_raw.z = 0;
  sensorData.magentic_raw.x = 0;
  sensorData.magentic_raw.y = 0;
  sensorData.magentic_raw.z = 0;
  sensorData.raw_timestamp = 0;
  sensorData.flex_0 = 0;
}

void loop() {

  // capture DMP calculated orienation
  if (imu.fifoAvailable()) {
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      imu.computeEulerAngles();
    
      sensorData.orientation_quaternion.x = imu.calcQuat(imu.qw);
      sensorData.orientation_quaternion.y = imu.calcQuat(imu.qx);
      sensorData.orientation_quaternion.z = imu.calcQuat(imu.qy);
      sensorData.orientation_quaternion.w = imu.calcQuat(imu.qz);

      sensorData.orientation_euler.x = imu.roll;
      sensorData.orientation_euler.y = imu.pitch;
      sensorData.orientation_euler.z = imu.yaw;

      sensorData.orientation_timestamp = imu.time;
    }
  }

  // capture acceleration raw, gyro raw, and magnetic raw
  if (imu.dataReady()) {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

    sensorData.accel_raw.x = imu.calcAccel(imu.ax);
    sensorData.accel_raw.y = imu.calcAccel(imu.ay);
    sensorData.accel_raw.z = imu.calcAccel(imu.az);
    sensorData.gyro_raw.x = imu.calcGyro(imu.gx);
    sensorData.gyro_raw.y = imu.calcGyro(imu.gy);
    sensorData.gyro_raw.z = imu.calcGyro(imu.gz);
    sensorData.magentic_raw.x = imu.calcMag(imu.mx);
    sensorData.magentic_raw.y = imu.calcMag(imu.my);
    sensorData.magentic_raw.z = imu.calcMag(imu.mz);

    sensorData.raw_timestamp = imu.time;
  }

  // capture flex sensors
  sensorData.flex_0 = analogRead(A0);

  //publish data
  root.clear();
  
  JsonObject orientationQuat = root.createNestedObject("orientation_quaternion");
  orientationQuat["x"] = sensorData.orientation_quaternion.x;
  orientationQuat["y"] = sensorData.orientation_quaternion.x;
  orientationQuat["z"] = sensorData.orientation_quaternion.x;
  orientationQuat["w"] = sensorData.orientation_quaternion.x;
  JsonObject orientationEuler = root.createNestedObject("orientation_euler");
  orientationEuler["x"] = sensorData.orientation_euler.x;
  orientationEuler["y"] = sensorData.orientation_euler.x;
  orientationEuler["z"] = sensorData.orientation_euler.x;
  root["orientation_timestamp"] = sensorData.orientation_timestamp;

  /*
  JsonObject accelRaw = root.createNestedObject("accel_raw");
  accelRaw["x"] = sensorData.accel_raw.x;
  accelRaw["y"] = sensorData.accel_raw.y;
  accelRaw["z"] = sensorData.accel_raw.z;
  JsonObject gyroRaw = root.createNestedObject("gyro_raw");
  gyroRaw["x"] = sensorData.gyro_raw.x;
  gyroRaw["y"] = sensorData.gyro_raw.y;
  gyroRaw["z"] = sensorData.gyro_raw.z;
  JsonObject magenticRaw = root.createNestedObject("magentic_raw");
  magenticRaw["x"] = sensorData.magentic_raw.x;
  magenticRaw["y"] = sensorData.magentic_raw.y;
  magenticRaw["z"] = sensorData.magentic_raw.z;
  root["raw_timestamp"] = sensorData.raw_timestamp;
  */
  
  JsonObject flexSensors = root.createNestedObject("flex_sensors");
  flexSensors["index"] = sensorData.flex_0; 
  
  serializeJson(root,Serial);
  Serial.print('\0');
  delay(10);
}
