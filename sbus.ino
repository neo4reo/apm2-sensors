#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

#define SBUS_HEADER_VALUE       0x0F
#define SBUS_FOOTER_VALUE       0x00

#define SBUS_PAYLOAD_LEN          23
#define SBUS_CH_MAX               16

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

void sbus_parse() {
    if ( sbus_data.failsafe_act ) {
        Serial.println("SBUS: failsafe activated!");
        return;      
    }
    if ( sbus_data.frame_lost ) {
        Serial.println("SBUS: frame lost");
        return;
    }

    uint16_t ch_data[SBUS_CH_MAX];
    ch_data[  0 ] = sbus_data.ch1;
    ch_data[  1 ] = sbus_data.ch2;
    ch_data[  2 ] = ( sbus_data.ch3_hi << 10 ) | sbus_data.ch3_lo;
    ch_data[  3 ] = sbus_data.ch4;
    ch_data[  4 ] = sbus_data.ch5;
    ch_data[  5 ] = ( sbus_data.ch6_hi <<  9 ) | sbus_data.ch6_lo;
    ch_data[  6 ] = sbus_data.ch7;
    ch_data[  7 ] = sbus_data.ch8;
    ch_data[  8 ] = ( sbus_data.ch9_hi <<  8 ) | sbus_data.ch9_lo;
    ch_data[  9 ] = sbus_data.ch10;
    ch_data[ 10 ] = sbus_data.ch11;
    ch_data[ 11 ] = ( sbus_data.ch12_hi << 7 ) | sbus_data.ch12_lo;
    ch_data[ 12 ] = sbus_data.ch13;
    ch_data[ 13 ] = sbus_data.ch14;
    ch_data[ 14 ] = ( sbus_data.ch15_hi << 6 ) | sbus_data.ch15_lo;
    ch_data[ 15 ] = sbus_data.ch16;
    
    Serial.print(" ");
    Serial.print(ch_data[0]);
    Serial.print(" ");
    Serial.print(ch_data[1]);
    Serial.print(" ");
    Serial.print(ch_data[2]);
    Serial.print(" ");
    Serial.print(ch_data[3]);
#if 0
    for ( int i = 0; i < SBUS_PAYLOAD_LEN; i++ ) {
        Serial.print(" ");
        Serial.print(sbus_data.buf[i], DEC);
    }
#endif
    Serial.println();
}

// read available bytes on the sbus uart and return true if any new data is read
bool sbus_read() {
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
            if ( input == SBUS_FOOTER_VALUE) {
                sbus_parse();
            }
            state = 0; 
        }
    }
    
    return new_data;
}
