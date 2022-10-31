#include <Adafruit_VL53L0X.h>
#include <ArduinoJson.h>
#include <JY901.h>
#include <math.h>
#include <PID_v1.h>
#include <Servo.h>
#include <WiFi.h>
#include <Wire.h>
#include "ESPAsyncWebServer.h"

// PID fun-ness
double Input, Output, Setpoint; //Input is sensor reading (current angle); Setpoint is desired angle; Output is what PID says angle should be
//double kp = 1.6, ki = 0.005, kd = 0;    // these parameters worked!
double kp = 1.6, ki = 0.005, kd = 0.0001;    // tuning parameters
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, P_ON_E, DIRECT);

// esc
int escPin = 14;

int escPwmMin = 62;
int escPwmMax = 120;
int escPwmMean = (escPwmMin + escPwmMax) / 2;

int minAngle = -5;
int maxAngle = 5;

int escSig = 0;
int escDelayMs = 5;

Servo ESC;

// PWM stuff
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 0;

// constants associated with IMU
float acc     [3];
float gyro    [3];
float angles  [3];
float mag     [3];
short dstatus [4];

float gpsHeight;
float gpsYaw;
float gpsVelocity;

float lpressure;
float laltitude;

// lidar
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
double lidar_height_mm = 38 + 36.971; // 38 from bottom of base to to center of bearing; 36.971 from center of bearing to lidar mount
double lidar = 0;

// wifi communication with matlab
const char* ssid = "Michaela-ESP-Access";  
const char* password = "123456789"; 

AsyncWebServer server(80);
DynamicJsonDocument jsonBuffer(256);

String smoothing = "weighted_average";
float lidar_weight = 0;
float lidar_var = 1;
float imu_var = 1;

void setup() 
{
  Serial.begin(115200);
  while (! Serial) {
    delay(1);
  }

  // ESC
  ESC.attach(escPin);
  ESC.write(20);
  for (int i=30; i>0; i--) 
  {
    Serial.print(i); Serial.println(" seconds to plug in ESC");
    delay(1000);
  }
//  ESC.write(0);
  delay(500);
  for (int i=10; i>0; i--) 
  {
    Serial.print(i); Serial.println(" seconds until motor starts moving");
    delay(1000);
  }
  ramp_gimbal_motor_to_speed(escPwmMean, 5);  // ramp-up to mean speed (middle), step of 5 pulses
  
  // setup IMU
  JY901.StartIIC();

  // setup lidar
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  lox.startRangeContinuous(); // start continuous ranging

  // PID
  Input = get_angle(angles);
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC); 
  myPID.SetOutputLimits(-255,255);

//  // WiFi
//  WiFi.softAP(ssid, password);
//  IPAddress IP = WiFi.softAPIP(); // IP address is 192.168.4.1
//  server.on("/data_processing",HTTP_POST,[](AsyncWebServerRequest * request){},
//    NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {
//
//      // Using the webwrite() command in matlab you can send a value that can be read and associated witha  specific action 
//      String msg = String((char *)data_in, len); // takes the given value 
////      Serial.print("received message: ");   Serial.println(msg);
//
//      // parse the json message. the format is {"theta1":[float]; "theta2":[float]; "pen":[int]}
//      DeserializationError error = deserializeJson(jsonBuffer, msg); 
//      if (error) {
//        request->send_P(200, "text/plain", "-1"); 
//        Serial.println("error in json parsing");
//        return;
//      }
//
//      lidar_weight = jsonBuffer["lidar_weight"];
//      lidar_var = jsonBuffer["lidar_var"];
//      imu_var   = jsonBuffer["imu_var"];
//      smoothing = jsonBuffer["smoothing"].as<String>();
//
//      request->send_P(200, "text/plain", "1"); 
//  });
//  
//  server.begin();  // Start server (needed)

  Serial.println("setup done");
  Serial.print("exiting setup in ");
  for (int i=5; i>0; i--) {
    Serial.print(i);Serial.print("... ");
    delay(1000);
  }
  Serial.print("Blast-off");
  Serial.println("");
} 

void loop() 
{
  // get the angle
  Input = get_angle(angles);
  myPID.Compute();

  set_gimbal_motor(Output);   // convert the value to a speed

  // print data for debugging
  Serial.print(" input_angle:"); Serial.print(Input);
  Serial.print(" pid_output:");  Serial.print(Output);
//  Serial.print(" escSig:");Serial.print(escSig);
//  Serial.print(" Angle_x:"); Serial.print(angles[0]); Serial.print(" Angle_y:"); Serial.print(angles[1]); Serial.print(" Angle_z:"); Serial.print(angles[2]); 
//  Serial.print(" Lidar:"); Serial.print(lidar); //Serial.print("\n"); 
  Serial.print("\n");
              
  delay(escDelayMs);
}


double get_angle( float (& angles) [3])
{
  read_lidar();
  get_angles(angles);
  
  float imu_angle = angles[1];
  float lidar_angle = lidar_to_angle(sign(imu_angle));
  if (isnan(lidar_angle)) {
    return imu_angle;
  }
//  Serial.print(" lidar angle:"); Serial.print(lidar_angle);
  if (smoothing.equals("weighted_average")) {
    return lidar_weight*lidar_angle + (1-lidar_weight)*imu_angle;
  } else if (smoothing.equals("var_average")) {
    return (lidar_angle/lidar_var + imu_angle/imu_var) / (1/lidar_var + 1/imu_var);
  } else {  // if invalid smoothing, just return angle from IMU
    return imu_angle;  
  }
}

double read_lidar()
{
  lidar = lox.readRange();
  return lidar;
}

int sign(float x) {
  if (x > 0) { return 1; }
  if (x < 0) { return -1; }
  return 0;
}

double lidar_to_angle(int sign) {
  return sign * acos(lidar/lidar_height_mm) * RAD_TO_DEG;
}

void get_angles(float (& angle) [3])
{
  JY901.GetAngle();
  angle[0] = (float)JY901.stcAngle.Angle[0]/32768*180;
  angle[1] = (float)JY901.stcAngle.Angle[1]/32768*180;
  angle[2] = (float)JY901.stcAngle.Angle[2]/32768*180;

}

void set_gimbal_motor(double angle)
{
  escSig = limited_map(angle);
  ESC.write(escSig);
}

void ramp_gimbal_motor_to_speed(int desiredSpeed, int speedJump)
{
  if (escSig < desiredSpeed) 
  {
    while (escSig < desiredSpeed) 
    {
      escSig += min(desiredSpeed - escSig, speedJump);
      Serial.print("escSig: "); Serial.println(escSig);
      ESC.write(escSig);
      delay(escDelayMs);
    }
  } else {
    // to do later, we're only ramping up now
  }
}

int limited_map(double angle)
{
  if (angle < minAngle){
    return escPwmMin;
  } else if (angle > maxAngle){
    return escPwmMax;
  } else {
    return map(angle, minAngle, maxAngle, escPwmMin, escPwmMax);
  }

} 
