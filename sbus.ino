#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

#define SBUS_HEADER_VALUE       0x0F
#define SBUS_FOOTER_VALUE       0x00

#define MAX_SBUS 25

void sbus_read() {
    static byte sbus_buf[MAX_SBUS];
    static byte state = 0;
    static byte counter = 0;
    byte input;
    while ( Serial2.available() >= 1 ) {
        input = Serial2.read();
        Serial.println(input, HEX);
    }
}

