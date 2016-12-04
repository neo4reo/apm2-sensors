#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

#define SBUS_HEADER_VALUE       0x0F
#define SBUS_FOOTER_VALUE       0x00

#define SBUS_PAYLOAD_LEN          23

#define SBUS_MIN_VALUE           172
#define SBUS_CENTER_VALUE        992
#define SBUS_MAX_VALUE          1811
#define SBUS_RANGE              1640
#define SBUS_HALF_RANGE          820
#define SBUS_QUARTER_RANGE       410

#define SBUS_CH_MAX 16

// Structure defining the contents of the SBUS data payload (23 bytes).
// Each of the channel fields (ch1:ch16) occupies 11 bytes.
typedef union {
    byte buf[SBUS_PAYLOAD_LEN];

    struct __attribute__ ((packed)) {
	uint32_t ch1          : 11;
	uint32_t ch2          : 11;
	uint32_t ch3_lo       : 10;

	uint32_t ch3_hi       :  1;
	uint32_t ch4          : 11;
	uint32_t ch5          : 11;
	uint32_t ch6_lo       :  9;

	uint32_t ch6_hi       :  2;
	uint32_t ch7          : 11;
	uint32_t ch8          : 11;
	uint32_t ch9_lo       :  8;

	uint32_t ch9_hi       :  3;
	uint32_t ch10         : 11;
	uint32_t ch11         : 11;
	uint32_t ch12_lo      :  7;

	uint32_t ch12_hi      :  4;
	uint32_t ch13         : 11;
	uint32_t ch14         : 11;
	uint32_t ch15_lo      :  6;

	uint32_t ch15_hi      :  5;
	uint32_t ch16         : 11;
	uint32_t ch17         :  1; // digital channel
	uint32_t ch18         :  1; // digital channel
	uint32_t frame_lost   :  1;
	uint32_t failsafe_act :  1;
    };
} SBUS_DATA_U;
    
static SBUS_DATA_U sbus_data;
static uint16_t sbus_ch_data[ SBUS_CH_MAX ];
uint16_t sbus_raw[MAX_CHANNELS];

void sbus_parse() {
    //Serial.println("sbus_parse()");
    if ( sbus_data.failsafe_act ) {
        //Serial.println("SBUS: failsafe activated!");
	// Note this would prevent us from parsing any receiver fail safe settings!
        return;      
    }
    if ( sbus_data.frame_lost ) {
        //Serial.println("SBUS: frame lost");
        return;
    }

    sbus_ch_data[  0 ] = sbus_data.ch1;
    sbus_ch_data[  1 ] = sbus_data.ch2;
    sbus_ch_data[  2 ] = ( sbus_data.ch3_hi << 10 ) | sbus_data.ch3_lo;
    sbus_ch_data[  3 ] = sbus_data.ch4;
    sbus_ch_data[  4 ] = sbus_data.ch5;
    sbus_ch_data[  5 ] = ( sbus_data.ch6_hi <<  9 ) | sbus_data.ch6_lo;
    sbus_ch_data[  6 ] = sbus_data.ch7;
    sbus_ch_data[  7 ] = sbus_data.ch8;
    sbus_ch_data[  8 ] = ( sbus_data.ch9_hi <<  8 ) | sbus_data.ch9_lo;
    sbus_ch_data[  9 ] = sbus_data.ch10;
    sbus_ch_data[ 10 ] = sbus_data.ch11;
    sbus_ch_data[ 11 ] = ( sbus_data.ch12_hi << 7 ) | sbus_data.ch12_lo;
    sbus_ch_data[ 12 ] = sbus_data.ch13;
    sbus_ch_data[ 13 ] = sbus_data.ch14;
    sbus_ch_data[ 14 ] = ( sbus_data.ch15_hi << 6 ) | sbus_data.ch15_lo;
    sbus_ch_data[ 15 ] = sbus_data.ch16;

#if 0    
    Serial.print(" ");
    Serial.print(sbus_ch_data[0]);
    Serial.print(" ");
    Serial.print(sbus_ch_data[1]);
    Serial.print(" ");
    Serial.print(sbus_ch_data[2]);
    Serial.print(" ");
    Serial.print(sbus_ch_data[3]);
    for ( int i = 0; i < SBUS_PAYLOAD_LEN; i++ ) {
        Serial.print(" ");
        Serial.print(sbus_data.buf[i], DEC);
    }
    Serial.println();
#endif

#if 0    
    for ( int i = 0; i < SBUS_CH_MAX; i++ ) {
        if ( ch_data[i] < SBUS_MIN_VALUE || ch_data[i] > SBUS_MAX_VALUE ) {
            Serial.print("Warning detected a problem with sbus packet data, skipping frame, ch = ");
            Serial.println(i);
            return;
        }
    }
#endif
    
    // copy sbus values to receiver_raw (just the first MAX_CHANNELS)
    for ( int i = 0; i < MAX_CHANNELS; i++ ) {
        sbus_raw[i] = sbus_ch_data[i];
    }
    
    sbus_raw2norm(sbus_raw, receiver_norm);
    // receiver_pwm is only for pwm receivers // pwm_norm2pwm(receiver_norm, receiver_pwm);
    
    if ( receiver_norm[CH_8] < 0.0 ) {
        // manual pass through requested, let's get it done right now
        sas_update( receiver_norm );
        mixing_update( receiver_norm, true /* ch1-6 */, true /* ch7 */, false /* no ch8 */ );
        pwm_update(); // for the outputs
    } else {
        // autopilot mode, but let's update the sas gain tuning channel if requested
        mixing_update( receiver_norm, false /* ch1-6 */, config.sas_ch7gain /* ch7 */, false /* no ch8 */ );
    }
    //Serial.print(sbus_raw[CH_1]);
    //Serial.print(" ");
    //Serial.println(receiver_norm[CH_1]);
}

// read available bytes on the sbus uart and return true if any new data is read
// when a full packet is read, send that to the parser which will push the new data into the various data
// structures and trigger and output of new actuator (currently PWM only) values.
bool sbus_process() {
    static byte state = 0;
    byte input;
    bool new_data = false;
    
    //Serial.print("state = ");
    //Serial.println(state);
    if ( state == 0 ) {
        // scan for start of frame
        while ( Serial2.available() > 0 ) {
            new_data = true;
            input = Serial2.read();
            if ( input == SBUS_HEADER_VALUE ) {
                state = 1;
                break;
            }
        }
    }
    if ( state == 1 ) {
        // fill in sbus frame (when enough bytes are available)
        if ( Serial2.available() >= SBUS_PAYLOAD_LEN ) {
            new_data = true;
     	    for ( int i = 0; i < SBUS_PAYLOAD_LEN; i++ ) {
                input = Serial2.read();
                //Serial.print(" ");
                //Serial.print(input, DEC);
		sbus_data.buf[i] = input;
	    }
            //Serial.println();
	    state = 2;
        }   
    }
    if  ( state == 2 ) {
        //Serial.println("here in state = 2");
        // end of frame
        if ( Serial2.available() > 0 ) {
            new_data = true;
            //Serial.println("bytes are available");
            input = Serial2.read();
            if ( input == SBUS_FOOTER_VALUE ) {
                sbus_parse();
                state = 0; 
            } else {
                //Serial.println("wrong sbus footer value, skipping ahead to next footer byte");
                input = Serial2.read();
                while ( Serial2.available() > 0 && input != SBUS_FOOTER_VALUE ) {
                    input = Serial2.read();
                }
                if ( input == SBUS_FOOTER_VALUE ) {
                   // set state to zero so we begin parsing the next new frame correctly
                   // (but something went wrong with this frame so don't parse the data/discard.)
                   state = 0;
                }
            }
        }
    }
    
    if ( new_data && !found_sbus ) {
        found_sbus = true;
    }
    
    return new_data;
}


// compute normalized command values from the raw sbus values
void sbus_raw2norm( uint16_t *raw, float *norm ) {
    for ( int i = 0; i < MAX_CHANNELS; i++ ) {
        // convert to normalized form
        if ( symmetrical[i] ) {
            // i.e. aileron, rudder, elevator
	    norm[i] = (float)((int)raw[i] - SBUS_CENTER_VALUE) / SBUS_HALF_RANGE;
        } else {
	    // i.e. throttle, flaps
	    norm[i] = (float)((int)raw[i] - SBUS_MIN_VALUE) / SBUS_RANGE;
        }
    }
}



