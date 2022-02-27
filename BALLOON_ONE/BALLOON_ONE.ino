#include <SoftwareSerial.h>
#include <Arduino_JSON.h>
#include <Adafruit_GPS.h>
#include <Printers.h>
#include <XBee.h>
#include "balloon_radio_addresses.h"
#include "functions.h"

#define LORA_RX 22
#define LORA_TX 23

// This Node is Node 2 = Balloon 2
int node_id = 2;
String balloon_number = "One";

//SoftwareSerial Serial1(12, 13); // RX, TX
SoftwareSerial xbeeSerial(2, 4);  // RX, TX
//HardwareSerial Serial1(1); // RX = 22, TX =23 Documents\Arduino\arduino-esp32\cores\esp32\hardware.cpp

Adafruit_GPS GPS(&Serial2); // RX=16, TX=17 Documents\Arduino\arduino-esp32\cores\esp32\hardware.cpp
XBee xbee = XBee();
//XBeeWithCallbacks xbee;
XBeeResponse response = XBeeResponse();

// StatusResponse Object
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
// Create Reusable response objects for responses we expect to handle
ZBRxResponse rx = ZBRxResponse();

//JSONVar jsonObject;
//JSONVar loraPayload;

String myString;
char data;

uint8_t gps_coord_payload[54];

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
  Serial1.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX); // LORA_RX: 22, LORA_TX: 23
  xbeeSerial.begin(9600);
  xbee.begin(xbeeSerial);
  GPS.begin(9600);


  // https://www.monocilindro.com/2016/03/28/reading-gps-data-using-arduino-and-a-u-blox-neo-6m-gps-receiver/
  // Link for PUBX Commands
  delay(100);
  GPS.println(F("c")); //VTG OFF
  delay(100);
  GPS.println(F("$PUBX,40,GSA,0,0,0,0*4E")); //GSA OFF
  delay(100);
  GPS.println(F("$PUBX,40,GSV,0,0,0,0*59")); //GSV OFF
  delay(100);
  GPS.println(F("$PUBX,40,GLL,0,0,0,0*5C")); //GLL OFF
  delay(1000);
  //  GPS.println(F("$PUBX,40,GGA,0,0,0,0*5A")); //GGA OFF
  //  delay(1000);
  //  GPS.println(F("$PUBX,40,RMC,0,0,0,0*47")); //GGA OFF
  //  delay(1000);
  /* Variables and Functions to set up Timer Interrupt
     Timer Interrupt used to send Temp every 20 seconds */
  //  timer = timerBegin(0, 80, true);
  //  timerAttachInterrupt(timer, &onTimer, true);
  //  timerAlarmWrite(timer, 16000000, true);
  //  timerAlarmEnable(timer);
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

  //  Serial1.listen();
  if (Serial1.available() > 0 ) {
    myString = Serial1.readString();
    String addrStr = getValue(myString, ',', 0);   //--> address
    String addr = getValue(addrStr, '=', 1);
    //Format of a received LoRa message: +RCV=<Address>,<Length>,<Data>,<RSSI>,<SNR>
    String isPing = getValue(myString, ',', 2);
    Serial.println(isPing);
    if (isPing == String("Ping")) {
      Serial.print("MyString: "); Serial.println(myString);
      Serial.print("Sending Pong back to: "); Serial.println(addr);
      String payload = "Pong";
      String loraPacket = "AT+SEND=" + String(addr) + "," + String(payload.length()) + "," + payload + "\r\n";
      Serial1.print(loraPacket); //--> Send Data
      Serial.println(loraPacket);
    }
    if (String(myString[0]) == "+" and String(myString[1]) == "R" and addr != "10") {
      //      Serial.print("MyString: "); Serial.println(myString);
      /*
        1. Parse Packet
         a. address b. data length c. data payload d. RSSI e. SNR
        2. Parse Data Payload into JSON Object
         a. Timestamp and b. route directon b2 to b3 or b3 to b2
        3. Repackage into XBee Packet
        4. Send Package Receive Ack back to sending LoRa Radio
        5. Forward XBee Packet to XBee on other Balloon Node
      */
      //      Serial.println();
      //      Serial.println("============ NEW PACKET =============");
      //      Serial.println();

      String json = getValue(myString, ',', 2);     //--> data
      // Extract Items from LoRa JSON Payload into new jsonObject
      json.replace("|", ",");
      loraPayload = JSON.parse(json);

      //      Serial.print("loraPayload: "); Serial.println(loraPayload);
      String uuid;
      uuid = loraPayload["U"];
      String id;
      id = loraPayload["ID"];
      String ts;
      ts = loraPayload["T"];
      ts.trim();
      String latitude;
      latitude = loraPayload["L"];     // lat of sending node
      String longitude;
      longitude = loraPayload["G"];   // lon of sending node

      String addrStr = getValue(myString, ',', 0);   //--> address
      String addr = getValue(addrStr, '=', 1);
      String dtl = getValue(myString, ',', 1);    //--> data length
      String rssi = getValue(myString, ',', 3);   //--> RSSI
      String snr = getValue(myString, ',', 4);    //--> SNR

      //      Serial.println("============ END PACKET =============");

      /* 1. Forward incoming Ground Station Waypoint Information */
      String str_loraPayload = JSON.stringify(loraPayload);
      sendWaypoint(1, str_loraPayload, 3);
      loraPayload = "";
      delay(6000);
      /* 2. Collect and Send Waypoint Data from this Node */
      String gpsPacket = getCoordinates(uuid);
      Serial.print("GPS Packet - End of rx_lora_packet: "); Serial.println(gpsPacket);
      sendWaypoint(1, gpsPacket, 3); /* Arguments 1 (XBee) or 2 (LoRa); payload, and radio number */
      gpsPacket = "";
      delay(650);
    }

    /* Forward Ground Station One Packet Information (WaypointGroundStation1: WPGS1)*/

    /* Fetch GPS Coordinates of this balloon's location and send to balloon 2 (NODE 3).
        function getCoordinates(String uuid);

    */
  }
}


void rx_packet() {

  xbee.readPacket();
  String rx_packet;
  if (xbee.getResponse().isAvailable()) {
    //    Serial.println("Received Packet");
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
      Serial1.print(ground_message); //--> Send Data
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

void sendStatusUpdate() {
  return;
}

String getCoordinates(String uuid) {
  //  Serial.print("UUID from getCoordinates()");Serial.println(uuid);
  char c;
  JSONVar groundGPSPayload;

  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  } GPS.parse(GPS.lastNMEA());
  if (GPS.fix) {
    while (!GPS.newNMEAreceived()) {
      c = GPS.read();
    } GPS.parse(GPS.lastNMEA());
    while (!GPS.newNMEAreceived()) {
      c = GPS.read();
    } GPS.parse(GPS.lastNMEA());
  }
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
  char _year[3];

  sprintf(_time, "%02d:%02d:%02d", GPS.hour, GPS.minute, GPS.seconds);
  sprintf(_year, "%02d", GPS.year);
  Serial.print("Year: ");Serial.println(_year);
  if (_year[0] == '0') {
    Serial.println("Wrong date yo!");
  }
  sprintf(_date, "20%02d-%02d-%02d", GPS.year, GPS.month, GPS.day);
  sprintf(_sats, "%02d", GPS.satellites);

  altitude = dtostrf(alt, 9, 2, charArrayAltitude);
  String strAltitude = String(altitude);
  strAltitude.trim();
  latitude = dtostrf(flat, 8, 5, charArrayLat);    // Convert Latitude from float to char string
  longitude = dtostrf(flon, 9, 5, charArrayLon);  // Convert Longitude from float to char string
  ptr_hdop = dtostrf(_hdop, 4, 2, charArrayHDOP);
  groundGPSPayload["U"] = uuid;
  groundGPSPayload["ID"] = String(node_id);
  groundGPSPayload["T"] = String(_date) + " " + String(_time);
  groundGPSPayload["A"] = strAltitude; // meters above sea level
  groundGPSPayload["L"] = latitude;
  groundGPSPayload["G"] = longitude;
  String packet = JSON.stringify(groundGPSPayload);
  return packet;
}

/* radio_protocol 1 is XBee and radio_protocol 2 is LoRa */
void sendWaypoint(int radio_protocol, String payload, int sendto_radio) {
  uint8_t xbee_payload[150];
  XBeeAddress64 radio;
  ZBTxRequest packet;
  if (sendto_radio == 1) {
    radio = radio1;
  } else if (sendto_radio == 2) {
    radio = radio2;
  } else if (sendto_radio == 3) {
    radio = radio3;
  } else if (sendto_radio == 4) {
    radio = radio4;
  } else if (sendto_radio == 5) {
    radio = radio5;
  } else if (sendto_radio == 6) {
    radio = radio6;
  } else if (sendto_radio == 7) {
    radio = radio7;
  } else if (sendto_radio == 8) {
    radio = radio8;
  } else {
    radio = addrBroadcast;
  }
  switch (radio_protocol) {
    case 1:
      // statements
      //      Serial.println("XBee Protocol - Loop in sendWaypoint()");
      for (int i = 0; i < payload.length(); i++) {
        char c = payload[i];
        xbee_payload[i] = (uint8_t)c;
        Serial.print(c);
      }
      Serial.println();

      packet = ZBTxRequest(radio, xbee_payload, payload.length());
      xbee.send(packet);
      Serial.println("Sending XBee Packet - sendWaypoint()");
      break;
    case 2:
      // statements
      Serial.println("LoRa Protocol");
      String loraPacket = "AT+SEND=" + String(sendto_radio) + "," + String(payload.length()) + "," + payload + "\r\n";
      Serial1.print(loraPacket); //--> Send Data
      Serial.println(loraPacket);
      break;
  }
}
