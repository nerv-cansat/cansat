#include <Arduino.h>
#include <ArduinoJson.h>
#include <CanSatKit.h>
#include <SD.h>
#include <Wire.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <VL53L1X.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Adafruit_AHTX0.h>
#include "RTClib.h"
#include <Servo.h>
#include <SPS30.h>



using namespace CanSatKit;

#define LED_BUZZ 9
#define GAS A0
#define ONE_WIRE_BUS 2 

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature dallas_temp(&oneWire);
RTC_DS3231 rtc;
BMP280 bmp;
VL53L1X tof;
LIS3MDL mag;
LSM6 imu;
Adafruit_AHTX0 aht;
Servo motor;
SPS30 PM25;
Radio radio(Pins::Radio::ChipSelect,
            Pins::Radio::DIO0,
            433.0,
            Bandwidth_125000_Hz,
            SpreadingFactor_9,
            CodingRate_4_8);

uint32_t armTime = 0;
bool armed = false;


int ledCounter = 0;
int ADC_VALUE;
int ITER; 
float VOLTAGE; 
float DUST; 
float AVG_DUST; 
const int chipSelect = 11;
int lastRead[10];
bool saturated = false;
int pos = 0;
int co2_level = 0;
byte co2_hex[]= {0xFE, 0x04, 0x00, 0x07, 0x00, 0x01, 0x94, 0x04};
byte recv[6];

int StrToHex(String hexadecimal)
{
  char str[4];
  hexadecimal.toCharArray(str, 4); 
  return (int) strtol(str, 0, 16);
}

String parseCO2(){
    byte d1 = recv[3];
    byte d2 = recv[4];
    return (String(d1, HEX)+String(d2, HEX));
}

void tof_init(){
  tof.setTimeout(500);
  if (!tof.init())
  {
    SerialUSB.println("Failed to detect and initialize sensor!");
    while (1);
  }   
  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(50000); 
  tof.startContinuous(50);
}

float get_rtc_volt(){
  unsigned int data[2];
  Wire.beginTransmission(0x54);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(0x54, 2);
  if(Wire.available() == 2)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }
  int raw_adc = ((data[0] & 0x0F) * 256) + data[1];
  float volt = (raw_adc/4096.00)*3.30;
  return volt;
}


void transmit(float T, double P, int distance, int co2, int humidity, float dust[4]) {
  Frame buff1;
  DynamicJsonDocument radio_1(254);
  DynamicJsonDocument radio_2(254);
  DynamicJsonDocument radio_3(254);
  DateTime dt = rtc.now();
  String date = String(dt.year())+"-"+String(dt.month())+"-"+String(dt.day())+"-"+String(dt.hour())+"-"+String(dt.minute())+"-"+String(dt.second());
  int voltage = (analogRead(A2)*0.983); //x100
  int gas = (analogRead(GAS)*0.97);
  int temp = T*100;
  int pressure = P*100;  
  
  radio_1["time"] = date; 
  radio_1["t"] = temp; //x100
  radio_1["p"] = pressure; //x100
  radio_1["d"] = distance;
  radio_1["v"] = voltage; //x100
  radio_1["co2"] = co2;
  radio_1["h"] = humidity; //x100 z poziomu funkcji read
  radio_1["g"] = gas; //x100
  

  int acc_x = (((imu.a.x) * 0.000061)*981.0); //x100
  int acc_y = (((imu.a.y) * 0.000061)*981.0); //x100
  int acc_z = (((imu.a.z) * 0.000061)*981.0); //x100

  int gyro_x = ((imu.g.x) * 0.7476); //x100
  int gyro_y = ((imu.g.y) * 0.7476); //x100
  int gyro_z = ((imu.g.z) * 0.7476); //x100

  int clockVoltage = get_rtc_volt()*100;
  
  radio_2["acc"]["x"] = acc_x;
  radio_2["acc"]["y"] = acc_y;
  radio_2["acc"]["z"] = acc_z;

  radio_2["gyro"]["x"] = gyro_x;
  radio_2["gyro"]["y"] = gyro_y;
  radio_2["gyro"]["z"] = gyro_z;

  radio_2["rtc"] = clockVoltage;
  

  serializeJson(radio_1, buff1);
  radio.transmit(buff1);

  File dataFile = SD.open("json.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(buff1);
    dataFile.close();
  }
  else {
    SerialUSB.println("sd nie ma pliku");
  }

  buff1.clear();
  serializeJson(radio_2, buff1);
  radio.transmit(buff1);

  dataFile = SD.open("json.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(buff1);
    dataFile.close();
  }
  else {
    SerialUSB.println("sd nie ma pliku");
  }

  buff1.clear();
  int particles[4];
  for(int i = 0; i<4; i++){
    particles[i] = dust[i]*100;
  }
  radio_3["t"] = temp;
  radio_3["p"] = pressure;
  radio_3["pm"]["1"] = particles[0];
  radio_3["pm"]["25"] = particles[1];  
  radio_3["pm"]["40"] = particles[2];
  radio_3["pm"]["10"] = particles[3];

  serializeJson(radio_3, buff1);
  radio.transmit(buff1);

  dataFile = SD.open("json.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(buff1);
    dataFile.close();
  }
  else {
    SerialUSB.println("sd nie ma pliku");
  }
  buff1.clear();

  radio.flush();
}


void setup() {
  SerialUSB.begin(9600);
  Serial.begin(9600); //co2
  Wire.begin();
  pinMode(GAS, INPUT);
  pinMode(A2, INPUT);
  pinMode(LED_BUZZ, OUTPUT);
  digitalWrite(LED_BUZZ, HIGH);
  SD.begin(11);
  dallas_temp.begin(); 
  radio.begin();
  radio.disable_debug();
  tof_init();
  PM25.begin();
  PM25.beginMeasuring();
  if (! rtc.begin()) {
    SerialUSB.println("Couldn't find RTC");
  }
  if (rtc.lostPower()) {
    SerialUSB.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));   
  }
  rtc.disable32K();
  if (! aht.begin()) {
    SerialUSB.println("Could not find AHT? Check wiring");
  }
  if(!bmp.begin()) {
    SerialUSB.println("BMP init failed!");
  } else {
    SerialUSB.println("BMP init success!");
  }
  if (!imu.init())
  {
    SerialUSB.println("Failed to detect and initialize IMU!");
  }
  imu.enableDefault();
  mag.enableDefault();
  motor.attach(4);
  bmp.setOversampling(16);
  delay(500);
  digitalWrite(LED_BUZZ, LOW);
  motor.writeMicroseconds(1000);
}


void co2_measure(){
    if(Serial.available() >=7){
        Serial.readBytesUntil(0xFE, recv, 7);
        int val = StrToHex(parseCO2());
        if(val>300) co2_level = val;
        
    }
}

int read_humidity(){
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  return humidity.relative_humidity*100;
}

void motorBraking(){
  for(int i = 1; i<11; i++){
    motor.writeMicroseconds((1000+i*100));
    delay(200);
  }
  delay(1000);
  motor.writeMicroseconds(1000);
  armed = false;
}


void loop() {
  uint32_t now = rtc.now().unixtime();
  while((analogRead(A2)*0.983)<620){ //voltage protection, low voltage halts the program at this point

  }
  int distance = tof.read();
  if((distance >= 3000) && armTime==0){
    armTime = now;
  }
  else if (distance<3000){
    armTime=0;
  }

  if(armTime!=0 && ((now-armTime)>180)){
    armed = true;
    armTime = 0;
    for(int i = 0; i<10; i++){
        digitalWrite(LED_BUZZ, HIGH);
        delay(50);
        digitalWrite(LED_BUZZ, LOW);
        delay(50);
    } 
  }


  if((distance<2000) && armed) motorBraking();
  
  ledCounter++;
  double T, P;
  float temp;
  float mass_concen[4];
  if(PM25.dataAvailable()) PM25.getMass(mass_concen);
  Serial.write(co2_hex, 8);
  bmp.measureTemperatureAndPressure(T, P);
  imu.read();
  dallas_temp.requestTemperatures();
  temp = dallas_temp.getTempCByIndex(0);
  co2_measure();  
  transmit(temp, P, distance, co2_level, read_humidity(), mass_concen);  
  if(ledCounter>200){
    digitalWrite(LED_BUZZ,HIGH);
    ledCounter = 0;
  }
  delay(700);
  digitalWrite(LED_BUZZ,LOW);
  
}
