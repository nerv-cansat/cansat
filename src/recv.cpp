#include <Arduino.h>
#include <CanSatKit.h>
#include <ArduinoJson.h>

using namespace CanSatKit;
Radio radio(Pins::Radio::ChipSelect,
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

void loop() {
  // prepare empty space for received frame
  // maximum length is maximum frame length + null termination
  // 255 + 1 byte = 256 bytes
  char data[256];

  radio.receive(data);
  DynamicJsonDocument recv(512);
  deserializeJson(recv, data);
  int rssi = radio.get_rssi_last();
  recv["rssi"] = rssi;
  serializeJson(recv, SerialUSB);
  SerialUSB.print("\n");

}
