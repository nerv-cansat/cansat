#include <Arduino.h>
#include <CanSatKit.h>
#include <ArduinoJson.h>

using namespace CanSatKit;
//tu sie odbiera 
// set radio receiver parameters - see comments below
// remember to set the same radio parameters in
// transmitter and receiver boards!
Radio radio(Pins::Radio::ChipSelect, //pierdoli
            Pins::Radio::DIO0,
            433.0,                  // frequency in MHz
            Bandwidth_125000_Hz,    // bandwidth - check with CanSat regulations to set allowed value
            SpreadingFactor_9,      // see provided presentations to determine which setting is the best
            CodingRate_4_8);        // see provided presentations to determine which setting is the best

void setup() {
  SerialUSB.begin(115200);

  // start radio module  
  radio.begin();
}
//test merge 

void loop() {
  // prepare empty space for received frame
  // maximum length is maximum frame length + null termination
  // 255 + 1 byte = 256 bytes
  char data[256];

  // receive data and save it to string
  radio.receive(data);
  DynamicJsonDocument recv(512);
  deserializeJson(recv, data);
  
  // get and print signal level (rssi)
 // SerialUSB.print("Received (RSSI = ");
  //SerialUSB.print(radio.get_rssi_last());
  //SerialUSB.print("): ");
  int rssi = radio.get_rssi_last();
  recv["rssi"] = rssi;
  float temp = recv["temp"];
  float pressure = recv["pressure"];
  String timestamp = recv["timestamp"];
  float volt = recv["volt"];
  int distance = recv["distance"];
  
  serializeJson(recv, SerialUSB);
  SerialUSB.print("\n");

  // print received message
 // SerialUSB.println(temp);
//  SerialUSB.println(pressure);
  //SerialUSB.println(timestamp);
  //SerialUSB.println(distance);
  //SerialUSB.println(volt);
}
