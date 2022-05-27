#define TINY_BME280_I2C

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "TinyBME280.h"
#include "RF24.h"
#include <avr/sleep.h>
//#include <avr/power.h>

#define NODE_2_MAIN_ADDR 0xF0F0F0F0A0LL
#define MAIN_2_NODE_ADDR 0xF0F0F0F0A1LL

#define NRF_POWER PIN_PA6
#define BME_POWER PIN_PA7
#define DONE PIN_PA6

tiny::BME280 sensor;
RF24 radio(PIN_PA4, PIN_PA5);

bool uses_tpl = true;


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


payload_t payload = {0, 0, 0, 0, 0, 0};


/* 32 bytes max
    future header???
    id           -> 0 - 255    1B
    msg type     -> 0 - 255    1B
    mg len       -> 0 - 96     7bits
    voltage      -> 0 - 511    9bit -> 1B (probably shifted a few mv)
    sleep length -> 0 - 16383  14bit (unknown size as of now)
    sleep unit   -> 0 - 3      2bit -> 2B
        
    0 status            -> 1B
    1 voltage           -> 2B (V without decimal point)
    3 sleep len         -> 4B (ms)
    7 temperature float -> 4B
    11 humidity float   -> 4B
    15 pressure float   -> 4B
*/


uint16_t getVCC() {
    /* ACV = (DACREF/256) * Vref
     * Vref = 1.024
     * DACREF = 255;
     * ACV = (255/256) * 1.024 = 1.02
     */

    /* ADCV = (MUXPOS/12^2) * REFSEL
     * MUXPOS = DACREF0 = ACV = 1.02
     * REFSEL = VDD
     * ADCV = (1.02/12^2) * VDD
     * VDD = 1.02 * (12^2/ADCV)
     * VDD.xx = 417792 / VDD
     */

    //AC
    VREF.CTRLA = VREF_AC0REFSEL_1V024_gc;   //set AC0 Vref to 1.024V
    AC0.DACREF = 0xFF;                      //set DACREF
    
    //ADC
    ADC0.MUXPOS  = ADC_MUXPOS_DACREF0_gc;                             //MUXPOS = ACV
    ADC0.CTRLC   = 0x11111000;                                        //VDD as ref
    ADC0.CTRLB   = ADC_PRESC_DIV16_gc;                                //precaler 5/16 = 312.5 KHz
    ADC0.COMMAND = ADC_MODE_SINGLE_12BIT_gc | ADC_START_IMMEDIATE_gc; //single 12bit and start now
    
    while (ADC0.STATUS); //wait for reading to finish

    return 417792/ADC0.RESULT;
}


void setup() {
    pinMode(DONE, OUTPUT);
    digitalWrite(DONE, LOW);
    payload.status = 100;   //id
    randomSeed(analogRead(PIN_PA7));
    payload.id = random(1, RANDOM_MAX - 1);

    payload.retries = 0;
    
    if (!radio.begin()) {
        while(true);
    }

    //add to status on success
    if (sensor.beginI2C(0x76))
        payload.status += 1;

    radio.setCRCLength(RF24_CRC_8);
    radio.setRetries(15, 15);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.
    radio.setChannel(52);

    //radio.setRetries(3, 15);
    //radio.setDataRate(RF24_250KBPS);
    //radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(sizeof(payload));
    radio.openWritingPipe(NODE_2_MAIN_ADDR);
    radio.openReadingPipe(1, MAIN_2_NODE_ADDR);
    radio.stopListening();  // put radio in TX mode

    //set sleep time
    payload.sleep_ms = 5 * 60 * 1000; //5 minutes
}

void loop() {
    //get voltage
    payload.voltage = getVCC();

    //get data from sensor
    payload.pres = sensor.readFixedPressure() / 100.0;
    payload.hum  = sensor.readFixedHumidity() / 1000.0;
    payload.temp = sensor.readFixedTempC()    / 100.0;

    //Send data
    while (payload.retries < 10) {
        if (radio.write(&payload, sizeof(payload)))
            break;
        payload.retries++;
    }
    digitalWrite(DONE, HIGH);
    delay(1000);
    payload.status += 1;    //this should never happen with TPL
}
