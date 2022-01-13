#include <Arduino.h>
#include <ArduinoJson.h>
#include <CanSatKit.h>
#include <SD.h>
#include <Wire.h>
#include <DS1307.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <VL53L1X.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Adafruit_AHTX0.h>

using namespace CanSatKit;

#define LED_BUZZ 9
#define GAS A0
#define ONE_WIRE_BUS 2 
#define PM25_MIN_VOLTAGE     600 
#define PM25_VREF           3300 
#define PM25_PIN_LED           8 
#define PM25_PIN_ANALOG        A4
#define PM25_MAX_ITERS        10 
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature dallas_temp(&oneWire);
DS1307 clock;
RTCDateTime dt;
BMP280 bmp;
VL53L1X tof;
LIS3MDL mag;
LSM6 imu;
Adafruit_AHTX0 aht;
Radio radio(Pins::Radio::ChipSelect,
            Pins::Radio::DIO0,
            433.0,
            Bandwidth_125000_Hz,
            SpreadingFactor_9,
            CodingRate_4_8);

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
  Wire.begin();
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

float average (int * array, int len) 
{
  long sum = 0L ;  
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;
}


float tof_measure(){
  lastRead[pos] = tof.read();
  pos++;
  if(pos>9){
      pos = 0;
      saturated = true;
  }
  delay(10);
  if (tof.timeoutOccurred()) { SerialUSB.print(" TIMEOUT"); }
   return(average(lastRead,10));
}

float computeDust() {
  digitalWrite(PM25_PIN_LED, HIGH);
  delayMicroseconds(280);
  ADC_VALUE = analogRead(PM25_PIN_ANALOG);
  digitalWrite(PM25_PIN_LED, LOW);
  VOLTAGE = (PM25_VREF / 1024.0) * ADC_VALUE * 11;
  if (VOLTAGE > PM25_MIN_VOLTAGE)
  {
    return (VOLTAGE - PM25_MIN_VOLTAGE) * 0.2;
  }
 
  return 0;
}

float iterateDust(){
   AVG_DUST = 0;
   ITER = 0;
 
   while (ITER < PM25_MAX_ITERS)
   {
     DUST = computeDust();
     if (DUST > 0)
     {
       AVG_DUST += DUST;
       ITER++;
       delay(50);
     }     
   }
   
   AVG_DUST /= PM25_MAX_ITERS;
   return AVG_DUST;
   
}


void transmit(float T, double P, int distance, int co2, int humidity, float dust) {
  Frame buff1;
  DynamicJsonDocument radio_1(254);
  DynamicJsonDocument radio_2(254);
  dt = clock.getDateTime();
  String date = String(dt.year)+"-"+String(dt.month)+"-"+String(dt.day)+"-"+String(dt.hour)+"-"+String(dt.minute)+"-"+String(dt.second);
  int voltage = (analogRead(A2)*0.841); //x100
  int gas = (analogRead(GAS)*0.97);
  int temp = T*100;
  int pressure = P*100;  
  int pm25 = dust*100;
  radio_1["time"] = date; 
  radio_1["t"] = temp; //x100
  radio_1["p"] = pressure; //x100
  radio_1["d"] = distance;
  radio_1["v"] = voltage; //x100
  radio_1["co2"] = co2;
  radio_1["h"] = humidity; //x100 z poziomu funkcji read
  radio_1["g"] = gas; //x100
  radio_1["p25"] = pm25; //x100
  

  int acc_x = (((imu.a.x) * 0.000061)*981.0); //x100
  int acc_y = (((imu.a.y) * 0.000061)*981.0); //x100
  int acc_z = (((imu.a.z) * 0.000061)*981.0); //x100

  int gyro_x = ((imu.g.x) * 0.7476); //x100
  int gyro_y = ((imu.g.y) * 0.7476); //x100
  int gyro_z = ((imu.g.z) * 0.7476); //x100
  
  radio_2["acc"]["x"] = acc_x;
  radio_2["acc"]["y"] = acc_y;
  radio_2["acc"]["z"] = acc_z;

  radio_2["gyro"]["x"] = gyro_x;
  radio_2["gyro"]["y"] = gyro_y;
  radio_2["gyro"]["z"] = gyro_z;
  
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
}


void setup() {
  SerialUSB.begin(9600);
  Serial.begin(9600); //co2
  pinMode(GAS, INPUT);
  pinMode(A2, INPUT);
  pinMode(LED_BUZZ, OUTPUT);
  pinMode(PM25_PIN_LED, OUTPUT);
  digitalWrite(PM25_PIN_LED, LOW);
  digitalWrite(LED_BUZZ, HIGH);
  clock.begin();
  //for setting the current date vvvvv
 // clock.setDateTime(__DATE__, __TIME__);
  SD.begin(11);
  dallas_temp.begin(); 
  radio.begin();
  radio.disable_debug();
  tof_init();
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
  bmp.setOversampling(16);
  delay(500);
  digitalWrite(LED_BUZZ, LOW);
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

void loop() {
  double T, P;
  float temp;
  Serial.write(co2_hex, 8);
  int distance = tof.read();
  bmp.measureTemperatureAndPressure(T, P);
  imu.read();
  dallas_temp.requestTemperatures();
  temp = dallas_temp.getTempCByIndex(0);
  co2_measure();
  transmit(temp, P, distance, co2_level, read_humidity(), iterateDust());  
  delay(700);
}
