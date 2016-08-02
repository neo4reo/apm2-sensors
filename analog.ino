const uint8_t MAX_ANALOG_INPUTS = 4;        // upto 16 supported on mega 2560

const uint8_t PITOT_ANALOG_PIN = A0;
const uint8_t CURRENT1_ANALOG_PIN = A1;     // A12 if power module port used
const uint8_t VOLTAGE1_ANALOG_PIN = A2;     // A13 if power module port used, but caution bug kills VCC sensing, fixed in later apm code.
// const uint8_t CURRENT2_ANALOG_PIN = A3;
// const uint8_t VOLTAGE2_ANALOG_PIN = A4;
const uint8_t VCC_ANALOG_PIN = 255;        // from libraries/AP_AnalogSource/AP_Analog_Source_Arduino.h

uint16_t analog[MAX_ANALOG_INPUTS];

void setup_analogs() {
    pinMode(PITOT_ANALOG_PIN, INPUT);
    pinMode(CURRENT1_ANALOG_PIN, INPUT);
    pinMode(VOLTAGE1_ANALOG_PIN, INPUT);
    //pinMode(CURRENT2_ANALOG_PIN, INPUT);
    //pinMode(VOLTAGE2_ANALOG_PIN, INPUT);
    pinMode(VCC_ANALOG_PIN, INPUT);
}
    
void read_analogs() {
    // Analog inputs update (values are averages of the internal readings since the last read()
    // These are 10 bit values (0-1023) but floating point averages, so let's scale them up to the
    // full 16 bit range (0-65535) or multiple by 2^6 (64) so we can convey some precision of 
    // the "averaged" value.
    
    uint16_t v = analogRead(VCC_ANALOG_PIN);
    if ( v > 0 ) {
         analog[0] = 1126400UL / v;
    }
    analog[1] = analogRead(PITOT_ANALOG_PIN);
    analog[2] = analogRead(VOLTAGE1_ANALOG_PIN);
    analog[3] = analogRead(CURRENT1_ANALOG_PIN);
    //analog[4] = analogRead(VOLTAGE2_ANALOG_PIN);
    //analog[5] = analogRead(CURRENT2_ANALOG_PIN);

    /*    
    vcc_average = 0.99 * vcc_average + 0.01 * (analog[5] / 1000.0);
    battery_voltage = (analog[1]/64.0) * (vcc_average/1024.0) * VOLT_DIV_RATIO;
    battery_amps = (((analog[2]/64.0) * (vcc_average/1024.0)) - CURR_AMPS_OFFSET) * CURR_AMP_PER_VOLT * 10;
    amps_sum += battery_amps * dt_millis * 0.0002778; // .0002778 is 1/3600 (conversion to hours)
    */
}
