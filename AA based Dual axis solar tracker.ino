#include <WiFi.h>
#include <ESP32Servo.h>
#include <ESP32Time.h>
#include <CayenneMQTTESP32.h>
#include <ESP32Servo.h>
#define rainDigital 13

#include "DHT.h"
#define DHTPIN 5    
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);



float Lon = -2.33 * DEG_TO_RAD,
      Lat = 51.37 * DEG_TO_RAD,
      elevation,
      azimuth;


      //wifi credentials
char ssid[] = "Clane";
char wifipassword[] = "Clane1290";

//MQTT credentials   
char username[] = "3d4ae720-ac21-11ec-9f5b-45181495093e";
char password[] = "6735c060e38c5bf1a159ba1084d2cb431c0ba5b6";
char clientID[] = "5985c540-ac21-11ec-9f5b-45181495093e";



int val;



#define servo1 1
#define servo2 2
#define RAIN 3
#define automod 4
#define tem 5
#define hum 6

#define elevation_servo 4
#define azimuth_servo   2

int sun_azimuth;
int sun_elevation;
bool automode = false;

String time_str, current_hour, current_minute, current_day, current_month, current_year;

Servo Azi_servo;
Servo Ele_servo;

void setup() {
  Cayenne.begin(username, password, clientID, ssid, wifipassword);
  Serial.begin(115200);
    pinMode(rainDigital,INPUT);
   dht.begin();


  StartTime();
  UpdateLocalTime();
  Azi_servo.attach(azimuth_servo);
  Ele_servo.attach(elevation_servo);
}

void loop() {
  Cayenne.loop();

  
  // All time values must not be in DST
  
  UpdateLocalTime();
  Serial.println(Update_DateTime());
  Serial.println(automode);
  
  if(automode == false) {

    int val = digitalRead(rainDigital);
    if (val == 1) {
  Calculate_Sun_Position(current_hour.toInt(), current_minute.toInt(), 0, current_day.toInt(), current_month.toInt(), current_year.toInt()); // parameters are HH:MM:SS DD:MM:YY start from midnight and work out all 24 hour positions.
  
  Azi_servo.write(map(sun_azimuth, 90, 270, 180, 0));        // Align to azimuth
  if (sun_elevation < 0) sun_elevation = 0; // Point at horizon if less than horizon
  Ele_servo.write(145 - sun_elevation);  // map(value, fromLow, fromHigh, toLow, toHigh)
  delay(1 * 3 * 1000); // Wait 1-minute then check position again
  }
  else if (val == 0)
  {
    Ele_servo.write(90);
  }
  }
   delay(1000);
}

CAYENNE_IN(automod){
  int value = getValue.asInt();
  if (value == 1){
    automode = true;
  }
  else{
    automode = false;
  }
}

CAYENNE_IN(servo1)
{
  if (automode == true){
  Azi_servo.write(getValue.asInt());
  }
}

CAYENNE_IN(servo2)
{
  if (automode == true){
  Ele_servo.write(getValue.asInt());
}
}

CAYENNE_OUT(RAIN)
{
   int rainDigitalVal = digitalRead(rainDigital);
   Cayenne.virtualWrite(RAIN, rainDigitalVal);
}

CAYENNE_OUT(tem)
{
   float temp = dht.readTemperature();
   Cayenne.virtualWrite(tem, temp);
}

CAYENNE_OUT(hum)
{
   float humi = dht.readHumidity();
   Cayenne.virtualWrite(hum, humi);
}

void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
  float T, JDate_frac, L0, M, e, C, L_true, f, R, GrHrAngle, Obl, RA, Decl, HrAngle;
  long JDate, JDx;
  int   zone = 0;  //Unused variable but retained for continuity 
  JDate      = JulianDate(year, month, day);
  JDate_frac = (hour + minute / 60. + second / 3600.0) / 24.0 - 0.5;
  T          = JDate - 2451545; T = (T + JDate_frac) / 36525.0;
  L0         = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * T, 360);
  M          = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * T, 360);
  e          = 0.016708617 - 0.000042037 * T;
  C          = DEG_TO_RAD * ((1.9146 - 0.004847 * T) * sin(M) + (0.019993 - 0.000101 * T) * sin(2 * M) + 0.00029 * sin(3 * M));
  f          = M + C;
  Obl        = DEG_TO_RAD * (23 + 26 / 60.0 + 21.448 / 3600. - 46.815 / 3600 * T);
  JDx        = JDate - 2451545;
  GrHrAngle  = 280.46061837 + (360 * JDx) % 360 + 0.98564736629 * JDx + 360.98564736629 * JDate_frac;
  GrHrAngle  = fmod(GrHrAngle, 360.0);
  L_true     = fmod(C + L0, 2 * PI);
  R          = 1.000001018 * (1 - e * e) / (1 + e * cos(f));
  RA         = atan2(sin(L_true) * cos(Obl), cos(L_true));
  Decl       = asin(sin(Obl) * sin(L_true));
  HrAngle    = DEG_TO_RAD * GrHrAngle + Lon - RA;
  elevation  = asin(sin(Lat) * sin(Decl) + cos(Lat) * (cos(Decl) * cos(HrAngle)));
  azimuth    = PI + atan2(sin(HrAngle), cos(HrAngle) * sin(Lat) - tan(Decl) * cos(Lat)); // Azimuth measured east from north, so 0 degrees is North
  sun_azimuth   = azimuth   / DEG_TO_RAD;
  sun_elevation = elevation / DEG_TO_RAD;
  Serial.println("Longitude and latitude " + String(Lon / DEG_TO_RAD, 3) + " " + String(Lat / DEG_TO_RAD, 3));
  Serial.println("Year\tMonth\tDay\tHour\tMinute\tSecond\tElevation\tAzimuth");
  Serial.print(String(year) + "\t" + String(month) + "\t" + String(day) + "\t" + String(hour - zone) + "\t" + String(minute) + "\t" + String(second) + "\t");
  Serial.println(String(elevation / DEG_TO_RAD, 0) + "\t\t" + String(azimuth / DEG_TO_RAD, 0));
}

long JulianDate(int year, int month, int day) {
  long JDate;
  int A, B;
  if (month <= 2) {year--; month += 12;}
  A = year / 100; B = 2 - A + A / 4;
  JDate = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
  return JDate;
}
 
////////////// WiFi, Time and Date Functions /////////////////
int StartWiFi(const char* ssid, const char* password) {
  int connAttempts = 0;
  Serial.print(F("\r\nConnecting to: ")); Serial.println(String(ssid));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED ) {
    delay(500); Serial.print(".");
    if (connAttempts > 20) {
      Serial.println("\nFailed to connect to a Wi-Fi network");
      return false;
    }
    connAttempts++;
  }
  Serial.print(F("WiFi connected at: "));
  Serial.println(WiFi.localIP());
  return true;
}

void StartTime() {
  configTime(0, 0, "1.in.pool.ntp.org", "time.nist.gov");
  setenv("TZ", "IST-5:30", 1); // Change for your location
  UpdateLocalTime();
}

void UpdateLocalTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  }
  //See http://www.cplusplus.com/reference/ctime/strftime/
  Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S"); // Displays: Saturday, June 24 2017 14:05:49
  char output[50];
  strftime(output, 50, "%a %d-%b-%y  (%H:%M:%S)", &timeinfo);
  time_str = output;
}

String GetTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time - trying again");
  }
  //See http://www.cplusplus.com/reference/ctime/strftime/
  //Serial.println(&timeinfo, "%a %b %d %Y %H:%M:%S"); // Displays: Saturday, June 24 2017 14:05:49
  char output[50];
  strftime(output, 50, "%d/%m/%y %H:%M:%S", &timeinfo); //Use %m/%d/%y for USA format
  time_str = output;
  Serial.println(time_str);
  return time_str; // returns date-time formatted like this "11/12/17 22:01:00"
}

String Update_DateTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time - trying again");
  }
  //See http://www.cplusplus.com/reference/ctime/strftime/
  char output[50];
  strftime(output, 50, "%H", &timeinfo);
  current_hour   = output;
  strftime(output, 50, "%M", &timeinfo);
  current_minute = output;
  strftime(output, 50, "%d", &timeinfo);
  current_day    = output;
  strftime(output, 50, "%m", &timeinfo);
  current_month  = output;
  strftime(output, 50, "%Y", &timeinfo);
  current_year   = output;
  Serial.println(time_str);
  return time_str; // returns date-time formatted like this "11/12/17 22:01:00"
}


//void test_azimuth() {
//  Azi_servo.write(90);  // Centre position
//  delay(500);
//  Azi_servo.write(60);  // Centre position
//  delay(500);
//  Azi_servo.write(120); // Centre position
//  delay(500);
//  Azi_servo.write(90);  // Centre position
//  delay(500);
//}
//
//void test_elevation() {
//  for (int a = 5; a < 145; a = a + 2) {
//    Ele_servo.write(a);  // Centre position
//    delay(30);
//  }
//  Ele_servo.write(145);  // Centre position
//  delay(1000);
//}
