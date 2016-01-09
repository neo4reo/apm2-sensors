#ifndef _APM2_CONFIG_H_INCLUDED
#define _APM2_CONFIG_H_INCLUDED


//////////////////////////////////////////////////////////////////////////
// This is a section for RC / PWM constants to be shared around the sketch
//////////////////////////////////////////////////////////////////////////

// Maximum number of input or output channels supported
// NUM_CHANNELS is defined in the RC module (as 8)
// Note to self: don't try to fight the 16 channel sbus battle until (if?) we drop PWM input support entirely
#define MAX_CHANNELS NUM_CHANNELS

// this is the hardware PWM generation rate
// note the default is 50hz and this is the max we can drive analog servos
// digital servos should be able to run at 200hz -- 250hz is getting up close to the theoretical maximum
// of a 100% duty cycle.  Advantage for running this at 200+hz with digital servos is we should catch commanded
// position changes slightly faster for a slightly more responsive system (emphasis on slightly)
// TODO: make this configurable via an external command.
#define DEFAULT_PWM_HZ 50


//////////////////////////////////////////////////////////////////////////
// config structure saved to eeprom
//////////////////////////////////////////////////////////////////////////

typedef struct {
    int version;
    
    /* hz for pwm output signal, 50hz default for analog servos, maximum rate is servo dependent:
       digital servos can usually do 200-250hz
       analog servos and ESC's typically require 50hz */
    uint16_t pwm_hz[MAX_CHANNELS];
    
    /* actuator gain (reversing) */
    float act_gain[MAX_CHANNELS];
    
    /* mixing modes */
    bool mix_autocoord;
    bool mix_throttle_trim;
    bool mix_flap_trim;
    bool mix_elevon;
    bool mix_flaperon;
    bool mix_vtail;
    bool mix_diff_thrust;

    /* mixing gains */
    float mix_Gac; // aileron gain for autocoordination
    float mix_Get; // elevator trim w/ throttle gain
    float mix_Gef; // elevator trim w/ flap gain
    float mix_Gea; // aileron gain for elevons
    float mix_Gee; // elevator gain for elevons
    float mix_Gfa; // aileron gain for flaperons
    float mix_Gff; // flaps gain for flaperons
    float mix_Gve; // elevator gain for vtail
    float mix_Gvr; // rudder gain for vtail
    float mix_Gtt; // throttle gain for diff thrust
    float mix_Gtr; // rudder gain for diff thrust
    
    /* sas modes */
    bool sas_rollaxis;
    bool sas_pitchaxis;
    bool sas_yawaxis;
    bool sas_ch7tune;

    /* sas gains */
    float sas_rollgain;
    float sas_pitchgain;
    float sas_yawgain;
    float sas_ch7gain;
} config_t;

extern config_t config;

extern uint16_t apm2_serial_number;

#endif /* _APM2_CONFIG_H_INCLUDED */
