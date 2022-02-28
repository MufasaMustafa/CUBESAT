// This was written from SD_Test_ESP32
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Arduino_JSON.h>
#include "read_write_functions.h"

JSONVar jsonObject;

void setup(){
    
    Serial.begin(115200);
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }
    jsonObject["U"] = "12d4";
    jsonObject["ID"] = "2";
    jsonObject["T"] = "2022-02-20 11:12:15";
    jsonObject["A"] = "415.2";
    jsonObject["L"] = "38.12345";
    jsonObject["G"] = "-78.12345";
    
    String waypoint = JSON.stringify(jsonObject);
    waypoint += "\r\n";
    /* c_str() casts a String to a const char* */
    appendFile(SD, "/waypoints.txt", waypoint.c_str());
}

void loop() {
  
}
