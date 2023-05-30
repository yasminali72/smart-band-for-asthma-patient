
#include <FB_Const.h>
#include <FB_Error.h>
#include <FB_Network.h>
#include <FB_Utils.h>
#include <Firebase.h>
#include <FirebaseESP32.h>
#include <FirebaseFS.h>
#include <Firebase_Client_Version.h>
#include <MB_File.h>
#include <MB_NTP.h>

#include <Wire.h> 

#include "MQ135.h" // mq
#define ANALOGPIN 4

#include "MAX30100_PulseOximeter.h" //heart rate
#define BLYNK_PRINT Serial
#include <Blynk.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
//#define REPORTING_PERIOD_MS 1000

#include <TinyGPSPlus.h> // gps
#define FIREBASE_HOST "https://esp8266-babdf-default-rtdb.firebaseio.com" 
#define FIREBASE_AUTH "AIzaSyAOxRkGAFP7FvWRloffCRDbxatLboBs8EI"
#define WIFI_SSID "Yasmin"
#define WIFI_PASSWORD "800159464@Y&A"

//dust
int measurePin = 2; //Connect dust sensor 
int ledPower = 25;   // dust sensor 
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;



PulseOximeter pox; //heart rate
int BPM, SpO2;
uint32_t tsLastReport = 0;

TinyGPSPlus gps;

FirebaseData firebaseData;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600);//gps
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Wire.begin();
  pox.begin();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
  
  pinMode(ledPower,OUTPUT);
  delay(500);
    
}
}
void loop() {
  //updateSerial();
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read())){
      displayInfo();
    }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
  digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(measurePin); // read the dust value
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); // turn the LED off
  delayMicroseconds(sleepTime);
  // 0 - 5V mapped to 0 - 1023 integer values
  // recover voltage
calcVoltage = voMeasured*(5.0/1024);
dustDensity = 0.17*calcVoltage-0.1;
  //if ( dustDensity < 0)
  //{
    //dustDensity = 0.00;
 // }
Serial.print("dustDensity="); 
Serial.print(dustDensity);
Serial.println("ug/m3");
Firebase.pushFloat(firebaseData, "dustDensity", dustDensity);

  // put your main code here, to run repeatedly:

// mq
float air_quality = analogRead(4);
Serial.print("Air Quality=");
Serial.print(air_quality);
Serial.print("PPM");
Serial.println();
Firebase.pushFloat(firebaseData, "Air/Quality", air_quality);


  pox.update();
 
 
    BPM = pox.getHeartRate();
    SpO2 = pox.getSpO2();
    if (millis() - tsLastReport > 5000)
  {
    
        Serial.print("HeartRate=");
        Serial.print(BPM);
        Serial.println("bpm");
        Serial.print("SpO2=");
        Serial.print(SpO2);
        Serial.println(" %");
        Firebase.pushInt(firebaseData, "/HeartRate", BPM);
        Firebase.pushInt(firebaseData, "/SpO2", SpO2);

   

    tsLastReport = millis();
        
    }
} 

void displayInfo()
{
  Serial.print(F("Location: "));
  if(gps.location.isValid()){
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    Serial.print("Lat= ");
    Serial.print(latitude, 6);
    Serial.print(F(","));
    Serial.print("Lng= ");
    Serial.print(longitude, 6);
    Serial.println();
    Firebase.pushDouble(firebaseData, "/gps/latitude", latitude);
      Firebase.pushDouble(firebaseData, "/gps/longitude", longitude);
     // Firebase.pushDouble(firebaseData, "/gps/altitude", altitude);
  }  
  else
  {
    Serial.println(F("INVALID"));
  }
}
void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}
