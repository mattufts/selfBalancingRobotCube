#include <Wire.h>
#include <JY901.h>

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

void setup() 
{
  Serial.begin(9600);
  JY901.StartIIC();
} 

void loop() 
{
  get_angle(angles);
  get_acc(acc);
  get_mag(mag);
  get_gyro(gyro);
  get_dstatus(dstatus);
  get_gps(gpsHeight, gpsYaw, gpsVelocity);
  get_pressure(lpressure, laltitude);
  
  Serial.print("Angle_x:"); Serial.print(angles[0]); Serial.print(" Angle_y:"); Serial.print(angles[1]); Serial.print(" Angle_z:"); Serial.print(angles[2]); Serial.print("\n"); 
  
  //print received data. Data was received in serialEvent;
            
//  JY901.GetLonLat();
//  Serial.print("Longitude:");Serial.print(JY901.stcLonLat.lLon/10000000);Serial.print("Deg");Serial.print((double)(JY901.stcLonLat.lLon % 10000000)/1e5);Serial.print("m Lattitude:");
//  Serial.print(JY901.stcLonLat.lLat/10000000);Serial.print("Deg");Serial.print((double)(JY901.stcLonLat.lLat % 10000000)/1e5);Serial.println("m");
//  
//  
//  Serial.println("");
  delay(10);
}

void get_acc(float (& acc) [3])
{
  JY901.GetAcc();
  acc[0] = (float)JY901.stcAcc.a[0]/32768*16;
  acc[1] = (float)JY901.stcAcc.a[1]/32768*16;
  acc[2] = (float)JY901.stcAcc.a[2]/32768*16;

}

void get_gyro(float (& gyro) [3])
{
  JY901.GetGyro();
  gyro[0] = (float)JY901.stcGyro.w[0]/32768*2000;
  gyro[1] = (float)JY901.stcGyro.w[1]/32768*2000;
  gyro[2] = (float)JY901.stcGyro.w[2]/32768*2000;

}

void get_angle(float (& angle) [3])
{
  JY901.GetAngle();
  angle[0] = (float)JY901.stcAngle.Angle[0]/32768*180;
  angle[1] = (float)JY901.stcAngle.Angle[1]/32768*180;
  angle[2] = (float)JY901.stcAngle.Angle[2]/32768*180;

}

void get_mag(float (& mag) [3])
{
  JY901.GetMag();
  mag[0] = (float)JY901.stcMag.h[0];
  mag[1] = (float)JY901.stcMag.h[1];
  mag[2] = (float)JY901.stcMag.h[2];

}

//String get_time()
//{
//  JY901.GetTime();
//  return (JY901.stcTime.ucYear + "-" + JY901.stcTime.ucMonth.toString() + "-" + JY901.stcTime.ucDay.toString() + " " + JY901.stcTime.ucHour + ":" + JY901.stcTime.ucMinute + ":" + JY901.stcTime.ucSecond + "." + JY901.stcTime.usMiliSecond/1000);
//
//}

void get_gps(float & gpsHeight, float & gpsYaw, float & gpsVelocity) 
{
  JY901.GetGPSV();
  gpsHeight =   (float)JY901.stcGPSV.sGPSHeight/10;
  gpsYaw =      (float)JY901.stcGPSV.sGPSYaw/10;
  gpsVelocity = (float)JY901.stcGPSV.lGPSVelocity/1000;  

}

void get_dstatus(short (&dstatus) [4]) 
{
  JY901.GetDStatus();
  dstatus[0] = (short)JY901.stcDStatus.sDStatus[0];
  dstatus[1] = (short)JY901.stcDStatus.sDStatus[1];
  dstatus[2] = (short)JY901.stcDStatus.sDStatus[2];
  dstatus[3] = (short)JY901.stcDStatus.sDStatus[3];
}

void get_pressure(float & lpressure, float & laltitude)
{
  JY901.GetPress();
  lpressure = JY901.stcPress.lPressure;
  laltitude = (float)JY901.stcPress.lAltitude/100;

}
