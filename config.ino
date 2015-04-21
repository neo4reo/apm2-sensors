#include <EEPROM.h>

#include "config.h"

#define CONFIG_VERSION 1

config_t config;  /* global definition */

int read_serial_number() {
     config.serial_number = 1;
     return 0;
};

void config_load_defaults() {
    Serial.printf("Setting default config ...\n");
    config.version = CONFIG_VERSION;
    config.serial_number = 0;
    mixing_defaults();
    sas_defaults();
}

int config_read_eeprom() {
    Serial.printf("Loading EEPROM...\n");
    int size = sizeof(config);
    if ( size > E2END - 2 /* checksum */ + 1 ) {
        return 0;
    }
    byte *ptr = (byte *)&config;
    for ( int i = 0; i < size; i++ ) {
        *ptr = EEPROM.read(i);
        Serial.printf("  %04d: %x\n", i, *ptr);
        ptr++;
    }
    byte read_cksum0 = EEPROM.read(size);
    byte read_cksum1 = EEPROM.read(size+1);
    byte calc_cksum0 = 0;
    byte calc_cksum1 = 0;
    ugear_cksum( START_OF_MSG0 /* arbitrary magic # */, START_OF_MSG1 /* arbitrary magic # */, (byte *)&config, size, &calc_cksum0, &calc_cksum1 );
    if ( read_cksum0 != calc_cksum0 || read_cksum1 != calc_cksum1 ) {
        Serial.printf("Check sum error ...\n");
        return 0;
    }
    return 1;
}

int config_write_eeprom() {
    Serial.printf("Write EEPROM...\n");
    int size = sizeof(config);
    if ( size > E2END - 2 /* checksum */ + 1 ) {
        return 0;
    }
    byte *ptr = (byte *)&config;
    for ( int i = 0; i < size; i++ ) {
        EEPROM.write(i, *ptr);
        Serial.printf("  %04d: %x\n", i, *ptr);
        ptr++;
    }
    byte calc_cksum0 = 0;
    byte calc_cksum1 = 0;
    ugear_cksum( START_OF_MSG0 /* arbitrary magic # */, START_OF_MSG1 /* arbitrary magic # */, (byte *)&config, size, &calc_cksum0, &calc_cksum1 );
    EEPROM.write(size, calc_cksum0);
    EEPROM.write(size+1, calc_cksum1);
    return 1;
}
