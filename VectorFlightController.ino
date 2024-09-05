/*
Project: VectorFlightController.ino
Author: Bora Ayhan (github.com/boraayhan)
Purpose: A preliminary flight computer system for an Arduino and sensor-based rocket.
Date: July 2024 - August 2024
*/
#include <Adafruit_MPU6050.h>
#include <SFE_BMP180.h>
#include <SD.h>
#include <Servo.h>
#include <dht.h>

//Thresholds and Params
#define PARACHUTE_DEPLOY_HEIGHT 70  //Height = 5.11*sqrt(mass of rocket / parachute area) * desired time before touchdown
#define HALT_THRESHOLD 0.5
#define MAX_ABORT_ITERATION 100
#define ENGINE_BURNOUT_TIME_MILLIS 1600
#define MOTION_ACCEL_THRESHOLD 16   //(Avg engine thrust / mass) - 9.8
#define MOTION_HEIGHT_THRESHOLD 10  //Be careful, height increases by 1 meter every 20 minutes due to sensor inaccuracy, meaning that 200 minutes of being armed will be detected as liftoff.

//Pins
#define DHT_PIN 5
#define IR_PIN 2
#define SD_CS_PIN 4
#define STROBE_PIN 7
#define BMP_PIN 3


//Attached components
File file;
Adafruit_MPU6050 mpu;
SFE_BMP180 bmp;
Servo s1;
dht DHT;

//Trackers
int state, abortIteration = 0;
bool ask, sensorError, sdError = false;

//Data
double xR, yR, zR, xA, yA, zA, in_temp, launch_time, altitude, out_temp, humidity, pressure, baseline = 0;

void setup() {
  Serial.println("starting");
  Serial.begin(115200);
  s1.attach(8);
  pinMode(STROBE_PIN, OUTPUT);
  Serial.println("pinmode");
  delay(1000);
  if (!mpu.begin()) {
    sensorError = true;
    Serial.println(F("MPU Error!"));
  }
  Serial.println("mpu");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  if (!bmp.begin()) {
    sensorError = true;
    Serial.println(F("BMP Error!"));
  }
  Serial.println("bmp");
  baseline = getPressure();
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD Card Hardware initialization failed!"));
  }
  file = SD.open(F("data.csv"));
  file.close();
  delay(400);
  file = SD.open(F("data.csv"), FILE_WRITE);
  if (file) {
    Serial.println(F("SD Card file functional!"));
    file.println(F("Time (s), State, Omega X (rad/s), Omega Y (rad/s), Omega Z (rad/s), Acceleration X (m/s^2), Acceleration Y (m/s^2), Acceleration Z (m/s^2), Internal Temperature (C), Altitude AGL (m), External Temperature (C), Humidity (%), External Pressure (hPA)"));
    Serial.println(F("Entered headers to data file!"));
  } else {
    sensorError = true;
    Serial.println(F("Error opening data file!"));
  }
  file.close();
  WriteString(F("Setup Complete!"));
  digitalWrite(STROBE_PIN, HIGH);
  delay(500);
  digitalWrite(STROBE_PIN, LOW);
}

void loop() {
  InterpretSerial(GetSerial());
  StateMachine();
  WriteData();
}

void GetMotion() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  xA = a.acceleration.y;
  yA = -a.acceleration.x;
  zA = -a.acceleration.z;

  xR = g.gyro.x;
  yR = g.gyro.y;
  zR = g.gyro.z;

  in_temp = temp.temperature;
  UpdatePressure();
  delay(100);
}

void GetSensors() {  //Lower priority than GetMotion
  int chk = DHT.read11(DHT_PIN);
  out_temp = DHT.temperature;
  humidity = DHT.humidity;
  delay(200);
}


void StateMachine() {
  GetMotion();
  switch (state) {
    case (0):  // Ground, safe
      digitalWrite(STROBE_PIN, LOW);
      break;
    case (1):  // Ground, armed
      digitalWrite(STROBE_PIN, HIGH);
      baseline = getPressure();  //Set "0m" to starting altitude
      //Center guidance system servos
      if (abs(yA) > MOTION_ACCEL_THRESHOLD || altitude > MOTION_HEIGHT_THRESHOLD) {  //Optimize this
        WriteString(F("Transitioning to State 2"));
        launch_time = millis();
        state = 2;
      }
      break;
    case (2):  // Propulsive Flight
      //Go easy on guidance systems to prevent overcompensation due to slight impefections in hardware
      digitalWrite(STROBE_PIN, LOW);
      if (millis() > launch_time + ENGINE_BURNOUT_TIME_MILLIS) {
        state = 3;
        WriteString(F("Transitioning to State 3"));
      }
      abortConfidence();
      break;
    case (3):  // Ballistic Ascent
      //Crazy on guidance systems
      if (descending()) {
        state = 4;
        WriteString(F("Transitioning to State 4"));
      }
      break;
    case (4):  // Ballistic Descent
      if (altitude < PARACHUTE_DEPLOY_HEIGHT) {
        //Deploy parachute
        state = 5;
        WriteString(F("Transitioning to State 5"));
      }
      break;
    case (5):  // Parachute
      if (abs(xR) + abs(yR) + abs(zR) < HALT_THRESHOLD) {
        WriteString(F("Transitioning to State 6"));
        state = 6;
      }
      break;
    case (6):  //Stopped, landed
      Blink();
      break;
    case (-1):  // Abort
      WriteString(F("[ABORT TEST] Aborted flight. Deploying chute."));
      //Deploy parachute
      state = 5;
      break;
  }
}

double getPressure() {  //hectopascals (hPa)
  char status;
  double T, P, p0, a;
  status = bmp.startTemperature();
  if (status != 0) {
    delay(status);
    status = bmp.getTemperature(T);
    if (status != 0) {
      status = bmp.startPressure(BMP_PIN);
      if (status != 0) {
        delay(status);
        status = bmp.getPressure(P, T);
        if (status != 0) {
          return (P);
        }
      }
    }
  }
}

void UpdatePressure() {
  pressure = getPressure();
  altitude = bmp.altitude(pressure, baseline);
}

void WriteData() {
  String error = "";
  GetMotion();
  GetSensors();
  double vars[] = { millis() / 1000.00, state, xR, yR, zR, xA, yA, zA, in_temp, altitude, out_temp, humidity, pressure };
  file = SD.open(F("data.csv"), FILE_WRITE);
  for (double entry : vars) {
    Serial.print(entry, 6);
    Serial.print(F(", "));
    if (file) {
      file.print(entry, 6);
      file.print(F(","));
    } else {
      error = F("Error writing file!");
    }
  }
  file.println();
  Serial.println(error);
  file.close();
}

bool descending() {
  double pAlt = altitude;
  delay(1000);
  GetMotion();
  return (pAlt > altitude);
}

int GetSerial() {
  String input = "";
  while (Serial.available()) {
    int c = Serial.read();
    input += String(c);
  }
  if (input.length() > 0) {
    Serial.println(input);
    return (input.toInt());
  }
  return -1;
}

void InterpretSerial(int s) {
  switch (s) {
    case (65827710):  //ARM
      state = 1;
      WriteString(F("Armed!"));
      break;
    case (677269677510):  //CHECK
      Serial.println(F("Inertial and Environmental Sensor Status:"));
      if (sensorError)
        Serial.println(F("FAULTY"));
      else
        Serial.println(F("FUNCTIONAL"));
      Serial.println(F("SD Card - Write Status:"));
      if (sdError)
        Serial.println(F("FAULTY"));
      else
        Serial.println(F("FUNCTIONAL"));
      break;
    case (8773806910):
      SD.remove(F("data.csv"));
      file = SD.open(F("data.csv"), FILE_WRITE);
      file.close();
      file = SD.open(F("data.csv"), FILE_WRITE);
      file.println(F("Time (s), State, Omega X (rad/s), Omega Y (rad/s), Omega Z (rad/s), Acceleration X (m/s^2), Acceleration Y (m/s^2), Acceleration Z (m/s^2), Internal Temperature (C), Altitude AGL (m), External Temperature (C), Humidity (%), External Pressure (hPA)"));
      file.close();
      Serial.println(F("All data on TF wiped!"));
      break;
  }
}

void WriteString(String s) {
  Serial.println(s);
  file = SD.open(F("data.csv"), FILE_WRITE);
  file.print(millis() / 1000.00, 6);
  file.print(F(","));
  file.println(s);
  file.print(F(","));
  file.print(altitude);
  file.print(F(","));
  file.print(F("SYS_MSG"));
  file.close();
}

void Blink() {
  digitalWrite(STROBE_PIN, HIGH);
  delay(400);
  digitalWrite(STROBE_PIN, LOW);
  delay(3000);
}


bool abortConfidence()  //This code may be dogshit but im pretty sure it works. How about averaging last k seconds of motion and comparing motion to pMotion?
{
  GetMotion();
  float c;
  c = (abs(xR) + abs(yR) + abs(zR)) / 100 + (abs(xA) + abs(zA)) / 100;  //Double check if rads or degrees. Threshold should be about 50 deg/s total.
  if (c >= 1) {
    WriteString(F("Aborted."));
    return true;
  } else if (c > 0.3) {
    abortIteration++;
    if (abortIteration < MAX_ABORT_ITERATION)  //Max iteration prevents infinite loop in flight
    {
      return abortConfidence();
    }
    abortIteration = 0;
    return false;
  } else {
    return false;
  }
}
