#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

#define SBUS_HEADER_VALUE       0x0F
#define SBUS_FOOTER_VALUE       0x00

#define SBUS_MAX                  25
#define SBUS_PACKET_SIZE          23


void sbus_parse( byte *buf ) {
    Serial.println("parse sbus packet");
}

void sbus_read() {
    static byte sbus_buf[SBUS_MAX];
    static byte state = 0;
    byte input;
    if ( state == 0 ) {
        // scan for start of frame
        while ( Serial2.available() > 0 ) {
            input = Serial2.read();
            if ( input == SBUS_HEADER_VALUE ) {
                state = 1;
                sbus_buf[0] = input;
                break;
            }
        }
    }
    if ( state == 1 ) {
        // fill in sbus frame (when enough bytes are available)
        if ( Serial2.available() >= SBUS_PACKET_SIZE ) {
     	    for ( int i = 0; i < SBUS_PACKET_SIZE; i++ ) {
		sbus_buf[i+1] = Serial2.read();
	    }
	    state = 2;
        }   
    }
    if  ( state == 2 ) {
        // end of frame
        if ( Serial2.available() > 0 ) {
            input = Serial2.read();
            sbus_buf[SBUS_MAX-1] = input;
            sbus_parse(sbus_buf);
            state == 0;
        }
    }
}
