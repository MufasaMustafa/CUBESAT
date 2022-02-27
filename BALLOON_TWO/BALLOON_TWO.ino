#include <SoftwareSerial.h>
#include <Arduino_JSON.h>
#include <Adafruit_GPS.h>
#include <XBee.h>
#include "balloon_radio_addresses.h"
#include "functions.h"
// This Node is Node 2 = Balloon 2
String node_id = "3";
String balloon_number = "Two";

SoftwareSerial ReyaxLoRa(12, 13); // RX, TX
SoftwareSerial xbeeSerial(2, 4);  // RX, TX

Adafruit_GPS GPS(&Serial2); // RX=16, TX=17 Documents\Arduino\arduino-esp32\cores\esp32\hardware.cpp
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();

// StatusResponse Object
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
// Create Reusable response objects for responses we expect to handle
ZBRxResponse rx = ZBRxResponse();

//JSONVar jsonObject;
//JSONVar loraPayload;

String myString;
char data;

uint8_t gps_coord_payload[60];

// for interrupts -- Tutorial for Using Timer Interrupts with ESP32
// https://www.visualmicro.com/page/Timer-Interrupts-Explained.aspx
// Variabes necessary for using ESP32 timer interrupt to send GPS Coord Packet and temperature updates
volatile int interrupts;
int totalInterrupts;


// Handle Interrupts
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ISR for ESP Interrupt
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interrupts++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {

  Serial.begin(9600);
  xbeeSerial.begin(9600);
  xbee.begin(xbeeSerial);
  ReyaxLoRa.begin(9600);
  GPS.begin(9600);


  // https://www.monocilindro.com/2016/03/28/reading-gps-data-using-arduino-and-a-u-blox-neo-6m-gps-receiver/
  // Link for PUBX Commands
  delay(100);
  GPS.println(F("$PUBX,40,VTG,0,0,0,0*5E")); //VTG OFF
  delay(100);
  GPS.println(F("$PUBX,40,GSA,0,0,0,0*4E")); //GSA OFF
  delay(100);
  GPS.println(F("$PUBX,40,GSV,0,0,0,0*59")); //GSV OFF
  delay(100);
  GPS.println(F("$PUBX,40,GLL,0,0,0,0*5C")); //GLL OFF
  delay(1000);
  /* Variables and Functions to set up Timer Interrupt
     Timer Interrupt used to send Temp every 20 seconds */
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 20000000, true);
  timerAlarmEnable(timer);
}

void loop() {

  rx_packet(); // digiMesh Packet Received
  rx_lora_packet(); // lora Packet Received

  if (interrupts > 0) {
    portENTER_CRITICAL(&timerMux);
    interrupts--;
    portEXIT_CRITICAL(&timerMux);
    totalInterrupts++;

    Serial.println("Interrupt Triggered");
  }
}

void rx_lora_packet() {

  JSONVar jsonObject;
  JSONVar jsonString;
  JSONVar loraPayload;
  JSONVar digiPayload;

  //  ReyaxLoRa.listen();
  if (ReyaxLoRa.available() > 0 ) {
    myString = ReyaxLoRa.readString();
    if (String(myString[0]) == "+" and String(myString[1]) == "R") {
      Serial.print("MyString: "); Serial.println(myString);
      /*
        1. Parse Packet
         a. address b. data length c. data payload d. RSSI e. SNR
        2. Parse Data Payload into JSON Object
         a. Timestamp and b. route directon b2 to b3 or b3 to b2
        3. Repackage into XBee Packet
        4. Send Package Receive Ack back to sending LoRa Radio
        5. Forward XBee Packet to XBee on other Balloon Node
      */
      Serial.println();
      Serial.println("============ NEW PACKET =============");
      Serial.println();

      String json = getValue(myString, ',', 2);     //--> data
      // Extract Items from LoRa JSON Payload into new jsonObject
      json.replace("|", ",");
      loraPayload = JSON.parse(json);

      Serial.print("loraPayload: "); Serial.println(loraPayload);
      String uuid;
      uuid = loraPayload["UUID"];
      String ts;
      ts = loraPayload["Timestamp"];
      String da;
      da = loraPayload["DestinationAddress"];
      String latitude;
      latitude = loraPayload["Latitude"];     // lat of sending node
      String longitude;
      longitude = loraPayload["Longitude"];   // lon of sending node

      String addrStr = getValue(myString, ',', 0);   //--> address
      String addr = getValue(addrStr, '=', 1);
      String dtl = getValue(myString, ',', 1);    //--> data length
      String rssi = getValue(myString, ',', 3);   //--> RSSI
      String snr = getValue(myString, ',', 4);    //--> SNR

      jsonObject["Timestamp"] = ts;
      jsonObject["rssi"] = rssi;
      jsonObject["snr"] = snr;
      jsonObject["SenderAddress"] = addr;
      jsonObject["Latitude"] = latitude;
      jsonObject["Longitude"] = longitude;
      jsonObject["DataLength"] = dtl;
      jsonObject["DestinationAddress"] = da;

      Serial.print("Timestamp: "); Serial.println(ts);
      Serial.print("Sender Address: "); Serial.println(addr);
      Serial.print("Destination Address: "); Serial.println(da);
      Serial.print("Latitude Ground Station One: "); Serial.println(latitude);
      Serial.print("Longitude Ground Station One: "); Serial.println(longitude);
      Serial.print("JSON length : "); Serial.println(dtl);
      Serial.print("JSON : "); Serial.println(json);
      Serial.print("RSSI : "); Serial.println(rssi);
      Serial.print("SNR : "); Serial.println(snr);
      
      if (da == "2") {
        
        char c;

        while (!GPS.newNMEAreceived()) {
          c = GPS.read();
        } GPS.parse(GPS.lastNMEA());
        while (!GPS.newNMEAreceived()) {
          c = GPS.read();
        } GPS.parse(GPS.lastNMEA());
        if (GPS.fix) {
          double _hdop = GPS.HDOP;
          double flat = GPS.latitudeDegrees;
          double flon = GPS.longitudeDegrees;
          double alt = GPS.altitude;
          char charArrayAltitude[15];
          char charArrayLat[15]; // Note that when declaring an array of type char, one more element that your initialization is required, to hold the required null character
          char charArrayLon[15];
          char *latitude;
          char *longitude;
          char *altitude;
          char *ptr_hdop;
          char charArrayHDOP[6];
          char hop_seq[7];
          char _time[9];
          char _date[11];
          char _sats[3];
          char _altitude[10];
          
          sprintf(_time, "%02d:%02d:%02d", GPS.hour, GPS.minute, GPS.seconds);
          sprintf(_date, "20%02d-%02d-%02d", GPS.year, GPS.month, GPS.day);
          sprintf(_sats, "%02d", GPS.satellites);
          
          altitude = dtostrf(alt, 9, 2, charArrayAltitude);
          String strAltitude = String(altitude);
          strAltitude.trim();
          latitude = dtostrf(flat, 8, 5, charArrayLat);    // Convert Latitude from float to char string
          longitude = dtostrf(flon, 9, 5, charArrayLon);  // Convert Longitude from float to char string
          ptr_hdop = dtostrf(_hdop, 4, 2, charArrayHDOP);
          digiPayload["id"] = uuid;
          digiPayload["b1_alt"] = strAltitude; // meters above sea level
          digiPayload["b1_lat"] = latitude;
          digiPayload["b1_lng"] = longitude;
          digiPayload["ts_b1"] = String(_date) + " " + String(_time);
//          String digiString = JSON.stringify(digiPayload);
          String digiString = String(addr)+","+node_id+","+String(uuid)+","+String(strAltitude)+","+String(latitude)+","+String(longitude)+","+String(_date)+" "+String(_time);
          int counter = 0;
          for (int i = 0; i < digiString.length(); i++) {
            char chr = (char)digiString[i];
            gps_coord_payload[i] = (int8_t)chr;
            Serial.print(chr);
            if (chr != ' ') {
              counter++;
            }
          }
          Serial.println();
          Serial.println(counter);
          //        gps_coord_payload[sizeof(gps_coord_payload)-1] = NULL;
          Serial.print("Size of gps payload: "); Serial.println(sizeof(gps_coord_payload));
          Serial.println();
          ZBTxRequest zbTx_gps_packet_b3 = ZBTxRequest(radio2, gps_coord_payload, sizeof(gps_coord_payload));
          xbee.send(zbTx_gps_packet_b3);
          Serial.println("Sending XBee Packet from Balloon Two to Balloon One");
          Serial.println();
          delay(1000);
          uint8_t debugPayload[] = "{\"message\":\"debug packet from Balloon Two\"}";
          Serial.print("Size of debug payload: "); Serial.println(sizeof(gps_coord_payload));
          ZBTxRequest zbTx_debug_packet_r1 = ZBTxRequest(radio1, debugPayload, sizeof(debugPayload));
          xbee.send(zbTx_debug_packet_r1);
          Serial.println("Sending XBee Packet from Balloon Two to Debug Radio 1");
          Serial.println();
          delay(1000);
        }
      }
      Serial.println();
      Serial.println("============ END PACKET =============");
    }
  }
}


void rx_packet() {

  xbee.readPacket();
  String rx_packet;
  if (xbee.getResponse().isAvailable()) {
    Serial.println("Received Packet");
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      xbee.getResponse().getZBRxResponse(rx);

      for (int i = 0; i < rx.getDataLength(); i++) {
        char c = rx.getData(i);
        Serial.print(c);
        rx_packet += char(c);
      }
      Serial.println();
      int fwd_pkt_length = rx_packet.length();
      Serial.print("Digi RX Packet to String: "); Serial.println(rx_packet);
      String ground_message;
      ground_message = "AT+SEND=4," + String(fwd_pkt_length) + "," + rx_packet + "\r\n";
      ReyaxLoRa.print(ground_message); //--> Send Data
      Serial.println(ground_message);
    }
  }
}

// I got this from : https://www.electroniclinic.com/reyax-lora-based-multiple-sensors-monitoring-using-arduino/
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void handle_gpsPacket(JSONVar gpsPacket) {

  String gpsString = "";
  uint8_t gps_coord_payload[100];

  // Read incoming packet to gpsString and gps_coord_payload char array
  Serial.print("Received GPS Packet to Forward: ");
  for (int i = 0; i < rx.getDataLength(); i++) {
    char c = rx.getData(i);
    gpsString.concat(c);
    gps_coord_payload[i] = (int8_t)c;
    Serial.print(c);
  }
  Serial.println();


  ZBTxRequest zbTx_gps_packet = ZBTxRequest(addrBroadcast, gps_coord_payload, sizeof(gps_coord_payload));
  xbee.send(zbTx_gps_packet);

}

void test_xbee() {
  uint8_t testPacket[] = "{\"message\":\"hello from balloon 2\"}";
  ZBTxRequest zbTx_gps_packet_b2 = ZBTxRequest(radio3, testPacket, sizeof(testPacket));
  xbee.send(zbTx_gps_packet_b2);
  Serial.println("Sending XBee Packet");
  delay(1000);
}
