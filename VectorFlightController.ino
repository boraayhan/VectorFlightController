#include <Adafruit_MPU6050.h>
#include <dht.h>
#include <SFE_BMP180.h>
#include <SD.h>

//Attached components
File file;
dht DHT;
Adafruit_MPU6050 mpu;
SFE_BMP180 bmp;
//Trackers
int state = 0;
bool ask, sensorError, sdError = false;

//Data
float xR, yR, zR, xA, yA, zA, in_temp, out_temp, launch_time, out_humid, altitude, pressure, baseline = 0;

void setup() {
  Serial.begin(9600);
  pinMode(12, OUTPUT);
  delay(1000);
  if (!mpu.begin()) {
    sensorError = true;
    Serial.println("MPU Error!");
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  if (!bmp.begin()) {
    sensorError = true;
    Serial.println("MPU Error!");
  }
  baseline = getPressure();
  if (!SD.begin(4)) {
    sdError = true;
  }
  file = SD.open("data_main.csv");
  file.close();
  if (!SD.exists("data_main.csv"))
    sdError = true;
  file = SD.open("data_main.csv", FILE_WRITE);
  file.println(F("Time (s), State, Omega X (rad/s), Omega Y (rad/s), Omega Z (rad/s), Acceleration X (m/s^2), Acceleration Y (m/s^2), Acceleration Z (m/s^2), Internal Temperature (C), External Temperature (C), Humidity (%), Altitude AGL (m), External Pressure (hPA)"));
  file.close();
}

void loop() {
  InterpretSerial(GetSerial());
  StateMachine();
  WriteData();
}
void GetMotion() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  xA = a.acceleration.x;
  yA = a.acceleration.y;
  zA = a.acceleration.z;

  xR = g.gyro.x;
  yR = g.gyro.y;
  zR = g.gyro.z;

  in_temp = temp.temperature;
  UpdatePressure();
  //write data to TF card
  delay(100);
}

void GetSensors()  //Lower priority than GetMotion
{
  int readData = DHT.read11(8);
  out_temp = DHT.temperature;
  out_humid = DHT.humidity;
  delay(200);
}

void StateMachine() {
  GetMotion();
  switch (state) {
    case (0):  // Ground, safe
      break;
    case (1):                    // Ground, armed
      baseline = getPressure();  //Set "0m" to starting altitude
      //Center fin servos
      if ((abs(xA) > 15 || abs(yA) > 15 || abs(zA) > 15) || altitude > 10) {  //Optimize this
        WriteString("State 2 (Launch)");
        launch_time = millis();
        state = 2;
      }
      break;
    case (2):  // Propulsive Flight
      //Go easy on fins

      if (millis() > launch_time + 1600) {
        state = 3;
        //Serial.println("State 3");
      }
      break;
    case (3):  // Ballistic Ascent
      //Crazy on fins
      if (descending()) {
        state = 4;
        WriteString("State 4");
      }
      break;
    case (4):  // Ballistic Descent
      if (altitude < 70) {
        //Deploy parachute
        state = 5;
        WriteString("State 5");
      }
      break;
    case (5):  // Parachute
      if (xR + yR + zR < 0.5) {
        WriteString("State 6");
        state = 6;
      }
      break;
    case (6):   //Stopped, landed
    case (-1):  // Abort
      break;
  }
}
long getPressure() {  //hectopascals (hPa)
  char status;
  double T, P, p0, a;
  status = bmp.startTemperature();
  if (status != 0) {
    delay(status);
    status = bmp.getTemperature(T);
    if (status != 0) {
      status = bmp.startPressure(3);
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
  GetMotion();
  GetSensors();
  float vars[] = { millis() / 1000, state, xR, yR, zR, xA, yA, zA, in_temp, out_temp, out_humid, altitude, pressure };
  file = SD.open("data_main.csv");
  for (float entry : vars) {
    if (file) {
      file.print(entry);
      file.print(F(","));
    }
  }
  file.println();
  file.close();
}

bool descending() {
  GetMotion();
  float pAlt = altitude;
  delay(500);
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
    case (65827710):
    //dont forget to actually arm lmfao
      WriteString("Armed!");
      break;
    case (698282798210):
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
  }
}

void WriteString(String s)
{
  Serial.println(s);
  file = SD.open("data_main.csv", FILE_WRITE);
  file.print("[NON-NUMERIC DATA],");
  file.print(millis()/1000);
  file.print(",");
  file.println(s);
  file.close();
}

void TestSD()
{
  file = SD.open("data_main.csv");
  file.write("SD Write Functional!");
  SD
  )
}