#include <FirebaseESP32.h>
#include <Wire.h> 

#include "MQ135.h" // mq
#define PIN_MQ135 35
MQ135 mq135_sensor(PIN_MQ135);

#include "MAX30100_PulseOximeter.h" //heart rate
#define REPORTING_PERIOD_MS     1000
#define BLYNK_PRINT Serial
#include <Blynk.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#define SDA_PIN 21
#define SCL_PIN 22
#define INT_PIN 19

#include <TinyGPSPlus.h> // gps
TinyGPSPlus gps;

#define FIREBASE_HOST "https://msensors-default-rtdb.firebaseio.com" 
#define FIREBASE_AUTH "AIzaSyDtXLFSkkjyh9qgSpDbrlxgREmiZ9dQrjg"
#define WIFI_SSID "Yasmin"
#define WIFI_PASSWORD "800159464@Y&A"

//dust
#include <GP2YDustSensor.h>

const uint8_t SHARP_LED_PIN = 25;   // Sharp Dust/particle sensor Led Pin
const uint8_t SHARP_VO_PIN = 34;    // Sharp Dust/particle analog out pin used for reading 
const float PM25_COEFFICIENT = 0.172;
const float PM10_COEFFICIENT = 0.283;
GP2YDustSensor dustSensor(GP2Y1010AU0F, SHARP_LED_PIN, SHARP_VO_PIN);



PulseOximeter pox; //heart rate
uint32_t tsLastReport = 0;


FirebaseData firebaseData;
void onBeatDetected()
{
    Serial.println("B:1");
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600);//gps
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
  delay(500);   
}
if (!pox.begin(PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT)) {
        Serial.println("ERROR: Failed to initialize pulse oximeter");
        for(;;);
    }

    pox.setOnBeatDetectedCallback(onBeatDetected);
}
void loop() {
  //updateSerial();
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();
    
  
  
//dust
int dustDensity = dustSensor.getDustDensity();
     if ( dustDensity < 0)
     {
         dustDensity = 0;
     }
float pm25 = PM25_COEFFICIENT * pow(dustDensity, 1.1);
float pm10 = PM10_COEFFICIENT * pow(dustDensity, 1.1);
  Serial.print("Dust density: ");
  Serial.println(dustDensity);
  Serial.print(" ug/m^3\t PM2.5 concentration: ");
  Serial.print(pm25);
  Serial.print(" µg/m³\t PM10 concentration: ");
  Serial.print(pm10);
  Serial.println(" µg/m³");
Firebase.pushFloat(firebaseData, "/datasensors/dustDensity", dustDensity);
Firebase.pushFloat(firebaseData, "/datasensors/pm2.5", pm25);
Firebase.pushFloat(firebaseData, "/datasensors/pm10", pm10);


// mq
float rzero = mq135_sensor.getRZero();
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();
float gas_concentration = (resistance / rzero) * ppm;
  Serial.print("MQ135 RZero: ");
  Serial.print(rzero);
  Serial.print("\t Resistance: ");
  Serial.print(resistance);
  Serial.print("\t PPM: ");
  Serial.print(ppm);
  Serial.println("ppm");
  Serial.print("gas_concentration:");
  Serial.println(gas_concentration);
Firebase.pushFloat(firebaseData, "/datasensors/RZero", rzero);
Firebase.pushFloat(firebaseData, "/datasensors/PPM", ppm);
Firebase.pushFloat(firebaseData, "/datasensors/resistance", resistance);
Firebase.pushFloat(firebaseData, "/datasensors/gasconcentration", gas_concentration);


//heart rate
pox.update();

    // Asynchronously dump heart rate and oxidation levels to the serial
    // For both, a value of 0 means "invalid"
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Serial.print("HeartRate:");
        Serial.println(pox.getHeartRate());
        Serial.print("SpO2:");
        Serial.println(pox.getSpO2());
        Firebase.pushInt(firebaseData, "/datasensors/HeartRate", pox.getHeartRate());
        Firebase.pushInt(firebaseData, "/datasensors/SpO2", pox.getSpO2());
        tsLastReport = millis();
    }
    

} 

void displayInfo()
{
  Serial.print(F("Location: "));
  if(gps.location.isValid())
  {
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    Serial.print("Lat= ");
    Serial.print(latitude, 6);
    Serial.print(F(","));
    Serial.print("Lng= ");
    Serial.print(longitude, 6);
    Serial.println();
    Firebase.pushDouble(firebaseData, "/datasensors/latitude", latitude);
    Firebase.pushDouble(firebaseData, "/datasensors/longitude", longitude);
  }  
  else
  {
    Serial.println(F("INVALID"));
    Firebase.pushDouble(firebaseData, "/datasensors/latitude", 0);
    Firebase.pushDouble(firebaseData, "/datasensors/longitude", 0);
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

