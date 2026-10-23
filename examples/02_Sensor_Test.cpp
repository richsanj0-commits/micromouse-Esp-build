#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define I2C_SDA 21
#define I2C_SCL 22

#define XSHUT_L  23
#define XSHUT_LF 19
#define XSHUT_F  18
#define XSHUT_RF 17
#define XSHUT_R  16

Adafruit_VL53L0X sensor_L;
Adafruit_VL53L0X sensor_LF;
Adafruit_VL53L0X sensor_F;
Adafruit_VL53L0X sensor_RF;
Adafruit_VL53L0X sensor_R;

bool status_L = false, status_LF = false, status_F = false, status_RF = false, status_R = false;

void setupSensors() {
  pinMode(XSHUT_L, OUTPUT); digitalWrite(XSHUT_L, LOW);
  pinMode(XSHUT_LF, OUTPUT); digitalWrite(XSHUT_LF, LOW);
  pinMode(XSHUT_F, OUTPUT); digitalWrite(XSHUT_F, LOW);
  pinMode(XSHUT_RF, OUTPUT); digitalWrite(XSHUT_RF, LOW);
  pinMode(XSHUT_R, OUTPUT); digitalWrite(XSHUT_R, LOW);
  delay(20);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(20);

  digitalWrite(XSHUT_L, HIGH); delay(10);
  status_L = sensor_L.begin(0x30);
  
  digitalWrite(XSHUT_LF, HIGH); delay(10);
  status_LF = sensor_LF.begin(0x31);
  
  digitalWrite(XSHUT_F, HIGH); delay(10);
  status_F = sensor_F.begin(0x32);
  
  digitalWrite(XSHUT_RF, HIGH); delay(10);
  status_RF = sensor_RF.begin(0x33);
  
  digitalWrite(XSHUT_R, HIGH); delay(10);
  status_R = sensor_R.begin(0x34);
}

int readSingleSensor(Adafruit_VL53L0X &sensor, bool isAvailable) {
  if (!isAvailable) return -1;
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) return (int)measure.RangeMilliMeter;
  return 8190;
}

void setup() {
  Serial.begin(115200);
  setupSensors();
  Serial.println("5x VL53L0X Setup Complete! Streaming...");
}

void loop() {
  int d_L  = readSingleSensor(sensor_L,  status_L);
  int d_LF = readSingleSensor(sensor_LF, status_LF);
  int d_F  = readSingleSensor(sensor_F,  status_F);
  int d_RF = readSingleSensor(sensor_RF, status_RF);
  int d_R  = readSingleSensor(sensor_R,  status_R);

  Serial.printf("[ToF mm] L:%4d | LF:%4d | F:%4d | RF:%4d | R:%4d\n", d_L, d_LF, d_F, d_RF, d_R);
  delay(150);
}
