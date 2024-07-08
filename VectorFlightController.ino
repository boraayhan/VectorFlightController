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
double xR, yR, zR, xA, yA, zA, in_temp, out_temp, launch_time, out_humid, altitude, pressure, baseline = 0;

void setup() {
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  delay(1000);
  if (!mpu.begin()) {
    sensorError = true;
    Serial.println(F("MPU Error!"));
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  if (!bmp.begin()) {
    sensorError = true;
    Serial.println(F("MPU Error!"));
  }
  baseline = getPressure();
  if (!SD.begin(6)) {
    Serial.println(F("SD Card initialization failed!"));
  }
  file = SD.open(F("data.csv"));
  file.close();
  delay(400);
  file = SD.open(F("data.csv"), FILE_WRITE);
  if (file) {
    Serial.println(F("SD Card file functional!"));
    file.println(F("Time (s), State, Omega X (rad/s), Omega Y (rad/s), Omega Z (rad/s), Acceleration X (m/s^2), Acceleration Y (m/s^2), Acceleration Z (m/s^2), Internal Temperature (C), External Temperature (C), Humidity (%), Altitude AGL (m), External Pressure (hPA)"));
    Serial.println(F("Entered headers to data file!"));
  } else {
    sensorError = true;
    Serial.println(F("Error opening data file!"));
  }
  file.close();
}

void loop() {
  InterpretSerial(GetSerial());
  StateMachine();
  WriteData();
}
void GetMotion() {  //Done
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

void GetSensors() {  //Lower priority than GetMotion - Done
  DHT.read11(8);
  out_temp = DHT.temperature;
  out_humid = DHT.humidity;
  delay(200);
}

void StateMachine() {
  GetMotion();
  switch (state) {
    case (0):  // Ground, safe
      digitalWrite(7, LOW);
      break;
    case (1):  // Ground, armed
      digitalWrite(7, HIGH);
      baseline = getPressure();  //Set "0m" to starting altitude
      //Center fin servos
      if ((abs(xA) > 15 || abs(yA) > 15 || abs(zA) > 15) || altitude > 10) {  //Optimize this
        WriteString(F("State 2 (Launch)"));
        launch_time = millis();
        state = 2;
      }
      break;
    case (2):  // Propulsive Flight
      //Go easy on fins

      if (millis() > launch_time + 1600) {
        state = 3;
        WriteString(F("State 3"));
      }
      break;
    case (3):  // Ballistic Ascent
      //Crazy on fins
      if (descending()) {
        state = 4;
        WriteString(F("State 4"));
      }
      break;
    case (4):  // Ballistic Descent
      if (altitude < 70) {
        //Deploy parachute
        state = 5;
        WriteString(F("State 5"));
      }
      break;
    case (5):  // Parachute
      if (xR + yR + zR < 0.5) {
        WriteString(F("State 6"));
        state = 6;
      }
      break;
    case (6):  //Stopped, landed
      Blink();
      break;
    case (-1):  // Abort
      break;
  }
}
double getPressure() {  //hectopascals (hPa), needs float adjustment
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
void UpdatePressure() {  //needs float adjustment
  pressure = getPressure();
  altitude = bmp.altitude(pressure, baseline);
}

void WriteData() {
  String error = "";
  GetMotion();
  GetSensors();
  double vars[] = { millis() / 1000.00, state, xR, yR, zR, xA, yA, zA, in_temp, out_temp, out_humid, altitude, pressure };
  file = SD.open(F("data.csv"), FILE_WRITE);
  for (double entry : vars) {
    Serial.print(entry, 6);
    Serial.print(", ");
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
  GetMotion();
  double pAlt = altitude;
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
      state = 1;
      //dont forget to actually arm lmfao
      WriteString(F("Armed!"));
      break;
    case (677269677510):
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
      file.println(F("Time (s), State, Omega X (rad/s), Omega Y (rad/s), Omega Z (rad/s), Acceleration X (m/s^2), Acceleration Y (m/s^2), Acceleration Z (m/s^2), Internal Temperature (C), External Temperature (C), Humidity (%), Altitude AGL (m), External Pressure (hPA)"));
      file.close();
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
  digitalWrite(7, HIGH);
  delay(400);
  digitalWrite(7, LOW);
  delay(3000);
}
