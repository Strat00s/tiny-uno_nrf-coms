/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

/**
 * A simple example of sending data from 1 nRF24L01 transceiver to another.
 *
 * This example was written to be used on 2 devices acting as "nodes".
 * Use the Serial Monitor to change each node's behavior.
 */
#include <SPI.h>
#include <Wire.h>
#include "RF24.h"
#include <LiquidCrystal_I2C.h>
#include "SdFat.h"


#define NODE_2_MAIN_ADDR 0xF0F0F0F0A0LL
#define MAIN_2_NODE_ADDR 0xF0F0F0F0A1LL

#define SD_CS   2
#define SD_MISO 3
#define SD_MOSI 4
#define SD_SCK  5
SoftSpiDriver<SD_MISO, SD_MOSI, SD_SCK> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi)
//#define LOGS "log.txt"
#define DATA "data.csv"

#define CE 9
#define CSN 10


SdFat sd;
File file;

RF24 radio(CE, CSN, 4000000); // using pin 7 for the CE pin, and pin 8 for the CSN pin

LiquidCrystal_I2C lcd(0x27, 16, 2);

typedef struct payload_t {
    uint32_t id;
    uint8_t status;
    uint8_t retries;
    uint16_t voltage;
    uint16_t sleep_ms;
    float temp;
    float hum;
    float pres;
} payload_t;

payload_t payload;

unsigned long message_timer = 0;
unsigned long msg_cnt = 0;
unsigned long second_timer = 0;
unsigned long sd_checker = 0;


bool sd_ok = false;
bool radio_ok = false;
bool id_ok = false;

//id, tbm, bytes, status, retries, voltage, sleep, temp, hum, pres
//2 files
//logs -> logs time, errors, messages and so on
//msg.csv -> received vaules



bool checkSD() {
    if (!sd.begin(SD_CONFIG)) {
        sd_ok = false;
        Serial.println(F("SD: SD check failed"));
    }
    else
        sd_ok = true;
    
    return sd_ok;
}

void checkFiles() {
    if (!checkSD())
        return;
    if (!sd.exists(DATA)) {
        Serial.println(F("FILES: 'data.csv' does not exist"));
        return;
    }

    Serial.println(F("FILES: Files ok"));
}

void logData(uint8_t bytes) {
    if (!checkSD())
        return;
    file.open(DATA, FILE_WRITE);
    file.seekEnd();

    file.print(millis());                          file.print(F(","));
    file.print(payload.id);                        file.print(F(","));
    file.print((millis() - message_timer) / 1000); file.print(F(","));
    file.print((int)bytes);                        file.print(F(","));
    file.print(payload.status);                    file.print(F(","));
    file.print(payload.retries);                   file.print(F(","));
    file.print(payload.voltage);                   file.print(F(","));
    file.print(payload.sleep_ms);                  file.print(F(","));
    file.print(payload.temp);                      file.print(F(","));
    file.print(payload.hum);                       file.print(F(","));
    file.println(payload.pres);

    //file.sync();
    file.close();
}

void printData(uint8_t bytes) {
    Serial.print(F("Status... "));
    if (payload.status > 101)
        Serial.println(F("FORBIDEN STATE"));
    else if (payload.status > 100)
        Serial.println(F("OK"));
    else
        Serial.println(F("Sensor init FAILED"));

    Serial.println(F("id, tbm, bytes, status, retries, voltage, sleep, temp, hum, pres"));
    Serial.print(millis());                          Serial.print(F(","));
    Serial.print(payload.id);                        Serial.print(F(","));
    Serial.print((millis() - message_timer) / 1000); Serial.print(F(","));
    Serial.print((int)bytes);                        Serial.print(F(","));
    Serial.print(payload.status);                    Serial.print(F(","));
    Serial.print(payload.retries);                   Serial.print(F(","));
    Serial.print(payload.voltage);                   Serial.print(F(","));
    Serial.print(payload.sleep_ms);                  Serial.print(F(","));
    Serial.print(payload.temp);                      Serial.print(F(","));
    Serial.print(payload.hum);                       Serial.print(F(","));
    Serial.println(payload.pres);
    Serial.println(F("--------------------------------------------------------------"));
}

void checkData() {
    if (!checkSD())
        return;
    file.open(DATA, FILE_READ);
    file.seekEnd();
    
    uint8_t nl_cnt = 0;
    char c = 0;
    while(nl_cnt != 2) {
        file.readBytes(&c, 1);
        if (c == '\r')
            nl_cnt++;
        file.seekCur(-2);
    }

    char id[12];
    sprintf(id, "%lu", payload.id);

    file.readStringUntil(',');
    String new_id = file.readStringUntil(',');

    Serial.print(F("PAYLOAD: ")); Serial.println(id);
    Serial.print(F("FILE: "));    Serial.println(new_id);

    if (!strcmp(id, new_id.c_str())) {
        Serial.println(F("SAME"));
        id_ok = true;
    }
    else
        id_ok = false;

    file.seekEnd();
    file.close();
}

void printLCD() {
    lcd.clear();
    if (payload.status > 101) {
        lcd.setCursor(14, 1);
        lcd.print(F("XX"));
    }
    else if (payload.status > 100) {
        lcd.setCursor(14, 1);
        lcd.print(F("OK"));
    }
    else {
        lcd.setCursor(14, 1);
        lcd.print(F("BD"));
    }

    lcd.setCursor(0, 0);
    lcd.print(payload.id);

    lcd.setCursor(11, 0);
    lcd.print((millis() - message_timer) / 1000);

    lcd.setCursor(0, 1);
    lcd.print(payload.voltage / 100);
    lcd.print(F("."));
    lcd.print(payload.voltage % 100);
    lcd.setCursor(5, 1);
    lcd.print(payload.status);
    lcd.print(F(" "));
    lcd.print(payload.retries);
    lcd.print(F(" "));
    lcd.print(sd_ok ? F("O") : F("X"));
    lcd.print(id_ok ? F("O") : F("X"));
}

void radioInit() {
    if (!radio.begin()) {
        Serial.println(F("NRF: INIT... FAILED"));
        radio_ok = false;
    }
    else {
        Serial.println(F("NRF: INIT... OK"));
        radio_ok = true;
    }

    radio.setCRCLength(RF24_CRC_8);
    radio.setRetries(15, 15);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.
    radio.setChannel(52);
    radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes
    radio.openWritingPipe(MAIN_2_NODE_ADDR);
    radio.openReadingPipe(1, NODE_2_MAIN_ADDR);
    radio.startListening();

    Serial.println(F("NRF: CONFIG... DONE"));
}


void setup() {
    pinMode(8, INPUT_PULLUP);

    Serial.begin(115200);
    Serial.println(F("RF24 SHORT"));

    if (!sd.begin(SD_CONFIG)) {
        Serial.println(F("SD: INIT... FAILED"));
        sd_ok = false;
    }
    else {
        Serial.println(F("SD: INIT... OK"));
        sd_ok = true;
    }

    lcd.init();
    lcd.backlight();
    Serial.println(F("LCD: INIT... OK"));

    radioInit();
}

void loop() {
    uint8_t pipe;
    if (radio.available(&pipe)) {             // is there a payload? get the pipe number that recieved it
        message_timer = millis();
        second_timer = millis();
        uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
        radio.read(&payload, bytes);            // fetch payload from FIFO
        
        logData(bytes);

        printData(bytes);
        printLCD();

        checkData();

        msg_cnt++;
    }

    if (millis() - second_timer>= 1000) {
        second_timer = millis();
        printLCD();
        if (!digitalRead(8))
            lcd.noBacklight();
        else
            lcd.backlight();
    }

    if (millis() - sd_checker >= 5000) {
        sd_checker = millis();
        checkSD();
    }
}
