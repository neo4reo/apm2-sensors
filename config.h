#ifndef _APM2_CONFIG_H_INCLUDED
#define _APM2_CONFIG_H_INCLUDED

typedef struct {
    int version;
    int serial_number;
    
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

#endif /* _APM2_CONFIG_H_INCLUDED */
