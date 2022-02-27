String getCoordinates(String uuid) {
//  Serial.print("UUID from getCoordinates()");Serial.println(uuid);
  char c;
  JSONVar groundGPSPayload;

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
    groundGPSPayload["U"] = uuid;
    groundGPSPayload["ID"] = String(node_id);
    groundGPSPayload["T"] = String(_date) + " " + String(_time);
    groundGPSPayload["A"] = strAltitude; // meters above sea level
    groundGPSPayload["L"] = latitude;
    groundGPSPayload["G"] = longitude;
    String packet = JSON.stringify(groundGPSPayload);
    packet.replace(",", "|");
    return packet;
  }
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
