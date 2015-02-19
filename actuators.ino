// Module to handle actuator input/output and mixing.

#define PWM_CENTER 1500
#define PWM_HALF_RANGE 450
#define PWM_RANGE (PWM_HALF_RANGE * 2)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)
#define PWM_MAX (PWM_CENTER + PWM_HALF_RANGE)

// SAS mode commands (format is cmd(byte), gain)
#define SAS_DEFAULTS 0
#define SAS_ROLLAXIS 1
#define SAS_PITCHAXIS 2
#define SAS_YAWAXIS 3
#define SAS_CH7_TUNE 10

bool sas_rollaxis = false;
bool sas_pitchaxis = false;
bool sas_yawaxis = false;
bool sas_ch7tune = false;

float sas_rollgain = 0.0;
float sas_pitchgain = 0.0;
float sas_yawgain = 0.0;
float sas_ch7gain = 2.0;

// Mix mode commands (format is cmd(byte), gain 1,, gain 2
#define MIX_DEFAULTS 0
#define MIX_AUTOCOORDINATE 1
#define MIX_THROTTLE_TRIM 2
#define MIX_FLAP_TRIM 3
#define MIX_ELEVONS 4
#define MIX_FLAPERONS 5
#define MIX_VTAIL 6
#define MIX_DIFF_THRUST 7

bool mix_autocoord = false;
bool mix_throttle_trim = false;
bool mix_flap_trim = false;
bool mix_elevon = false;
bool mix_flaperon = false;
bool mix_vtail = false;
bool mix_diff_thrust = false;

float mix_Gac = 0.5; // aileron gain for autocoordination
float mix_Get = -0.1; // elevator trim w/ throttle gain
float mix_Gef = 0.1; // elevator trim w/ flap gain

float mix_Gea = 1.0; // aileron gain for elevons
float mix_Gee = 1.0; // elevator gain for elevons
float mix_Gfa = 1.0; // aileron gain for flaperons
float mix_Gff = 1.0; // flaps gain for flaperons
float mix_Gve = 1.0; // elevator gain for vtail
float mix_Gvr = 1.0; // rudder gain for vtail
float mix_Gtt = 1.0; // throttle gain for diff thrust
float mix_Gtr = 0.1; // rudder gain for diff thrust


// define if a channel is symmetrical or not (i.e. mapped to [0,1] for throttle, flaps, spoilers; [-1,1] for aileron, elevator, rudder
bool symmetrical[NUM_CHANNELS] = {1, 1, 0, 1, 0, 0, 0, 0};

// official flight command values.  These could source from the RC receiver or the autopilot depending on the auto/manual
// selection switch state.  These are pre-mix commands and will be mixed and written to the actuators for both manual and
// autonomous flight modes.
float aileron_cmd = 0.0;
float elevator_cmd = 0.0;
float throttle_cmd = 0.0;
float rudder_cmd = 0.0;
float gear_cmd = 0.0;
float flap_cmd = 0.0;
float ch7_cmd = 0.0;
float ch8_cmd = 0.0;


// reset sas parameters to startup defaults
void sas_defaults() {
    sas_rollaxis = false;
    sas_pitchaxis = false;
    sas_yawaxis = false;
    sas_ch7tune = false;

    sas_rollgain = 0.0;
    sas_pitchgain = 0.0;
    sas_yawgain = 0.0;
    sas_ch7gain = 2.0;
};


// reset mixing parameters to startup defaults
void mixing_defaults() {
    mix_autocoord = false;
    mix_throttle_trim = false;
    mix_flap_trim = false;
    mix_elevon = false;
    mix_flaperon = false;
    mix_vtail = false;
    mix_diff_thrust = false;

    mix_Gac = 0.5; // aileron gain for autocoordination
    mix_Get = -0.1; // elevator trim w/ throttle gain
    mix_Gef = 0.1; // elevator trim w/ flap gain

    mix_Gea = 1.0; // aileron gain for elevons
    mix_Gee = 1.0; // elevator gain for elevons
    mix_Gfa = 1.0; // aileron gain for flaperons
    mix_Gff = 1.0; // flaps gain for flaperons
    mix_Gve = 1.0; // elevator gain for vtail
    mix_Gvr = 1.0; // rudder gain for vtail
    mix_Gtt = 1.0; // throttle gain for diff thrust
    mix_Gtr = 0.1; // rudder gain for diff thrust
};


bool sas_command_parse(byte *buf) {
    bool enable = buf[1];
    uint8_t lo, hi;
    uint16_t val;

    lo = buf[2];
    hi = buf[3];
    val = hi*256 + lo; 
    float gain = ((float)val - 32767.0) / 10000.0;

    if ( buf[0] == SAS_DEFAULTS ) {
        sas_defaults();
    } else if ( buf[0] == SAS_ROLLAXIS ) {
        sas_rollaxis = enable;
        sas_rollgain = gain;
    } else if ( buf[0] == SAS_PITCHAXIS ) {
        sas_pitchaxis = enable;
        sas_pitchgain = gain;
    } else if ( buf[0] == SAS_YAWAXIS ) {
        sas_yawaxis = enable;
        sas_yawgain = gain;
    } else if ( buf[0] == SAS_CH7_TUNE ) {
        sas_ch7tune = enable;
    } else {
        return false;
    }
    
    return true;
}


bool mixing_command_parse(byte *buf) {
    bool enable = buf[1];
    uint8_t lo, hi;
    uint16_t val;

    lo = buf[2];
    hi = buf[3];
    val = hi*256 + lo; 
    float g1 = ((float)val - 32767.0) / 10000.0;

    lo = buf[4];
    hi = buf[5];
    val = hi*256 + lo;    
    float g2 = ((float)val - 32767.0) / 10000.0;
    
    if ( buf[0] == MIX_DEFAULTS ) {
        mixing_defaults();
    } else if ( buf[0] == MIX_AUTOCOORDINATE ) {
        mix_autocoord = enable;
        mix_Gac = g1;
    } else if ( buf[0] == MIX_THROTTLE_TRIM ) {
        mix_throttle_trim = enable;
        mix_Get = g1;
    } else if ( buf[0] == MIX_FLAP_TRIM ) {
        mix_flap_trim = enable;
        mix_Gef = g1;
    } else if ( buf[0] == MIX_ELEVONS ) {
        mix_elevon = enable;
        mix_Gea = g1;
        mix_Gee = g2;
    } else if ( buf[0] == MIX_FLAPERONS ) {
        mix_flaperon = enable;
        mix_Gfa = g1;
        mix_Gff = g2;
    } else if ( buf[0] == MIX_VTAIL ) {
        mix_vtail = enable;
        mix_Gve = g1;
        mix_Gvr = g2;
    } else if ( buf[0] == MIX_DIFF_THRUST ) {
        mix_diff_thrust = enable;
        mix_Gtt = g1;
        mix_Gtr = g2;
    } else {
        return false;
    }
    
    return true;
}


// compute normalized command values from the raw pwm values
void raw2norm( uint16_t raw[NUM_CHANNELS], float norm[NUM_CHANNELS] ) {
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        // convert to normalized form
        if ( symmetrical[i] ) {
            // i.e. aileron, rudder, elevator
	    norm[i] = (float)((int)raw[i] - PWM_CENTER) / PWM_HALF_RANGE;
        } else {
	    // i.e. throttle, flaps
	    norm[i] = (float)((int)raw[i] - PWM_MIN) / PWM_RANGE;
        }
    }
}


// compute raw pwm values from normalized command values
void norm2raw( float norm[NUM_CHANNELS], uint16_t raw[NUM_CHANNELS] ) {
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        // convert to pulse length (special case ch6 when in flaperon mode)
        if ( symmetrical[i] || (i == 5 && mix_flaperon) ) {
            // i.e. aileron, rudder, elevator
	    raw[i] = PWM_CENTER + (int)(PWM_HALF_RANGE * norm[i]);
        } else {
	    // i.e. throttle, flaps
	    raw[i] = PWM_MIN + (int)(PWM_RANGE * norm[i]);
        }
        if ( raw[i] < PWM_MIN ) {
            raw[i] = PWM_MIN;
        }
        if ( raw[i] > PWM_MAX ) {
            raw[i] = PWM_MAX;
        }
    }
}


// compute the sas compensation in normalized 'command' space so that we can do proper output channel mixing later
void sas_update( float control_norm[NUM_CHANNELS] ) {
    // mixing modes that work at the 'command' level (before actuator value assignment)
    
    float tune = 1.0;
    if ( sas_ch7tune ) {
        tune = sas_ch7gain * receiver_norm[6];
        if ( tune < 0.0 ) {
            tune = 0.0;
        } else if ( tune > 2.0 ) {
            tune = 2.0;
        }
    } 
    if ( sas_rollaxis ) {
        control_norm[0] += tune * sas_rollgain * imu_sensors[0];
    }
    if ( sas_pitchaxis ) {
        control_norm[1] += tune * sas_pitchgain * imu_sensors[1];
    }
    if ( sas_yawaxis ) {
        control_norm[3] += tune * sas_yawgain * imu_sensors[2];
    }
}


// compute the actuator (servo) values for each channel.  Handle all the requested mixing modes here.
void mixing_update( float control_norm[NUM_CHANNELS], bool do_ch1_7, bool do_ch8 ) {
    if ( do_ch1_7 ) {
        aileron_cmd = control_norm[0];
        elevator_cmd = control_norm[1];
        throttle_cmd = control_norm[2];
        rudder_cmd = control_norm[3];
        gear_cmd = control_norm[4];
        flap_cmd = control_norm[5];
        ch7_cmd = control_norm[6];
    }
    if ( do_ch8 ) {
        ch8_cmd = control_norm[7];
    }
        
    // mixing modes that work at the 'command' level (before actuator value assignment)
    if ( mix_autocoord ) {
        rudder_cmd += mix_Gac * aileron_cmd;
    }
    if ( mix_throttle_trim ) {
        elevator_cmd += mix_Get * throttle_cmd;
    }
    if ( mix_flap_trim ) {
        elevator_cmd += mix_Gef * flap_cmd;
    }
  
    // copy default assignments as if no mixing
    if ( do_ch1_7 ) {
        actuator_norm[0] = aileron_cmd;
        actuator_norm[1] = elevator_cmd;
        actuator_norm[2] = throttle_cmd;
        actuator_norm[3] = rudder_cmd;
        actuator_norm[4] = gear_cmd;
        actuator_norm[5] = flap_cmd;
        actuator_norm[6] = ch7_cmd;
    }
    if ( do_ch8 ) {
        actuator_norm[7] = ch8_cmd;
    }
    
    if ( do_ch1_7 ) {
        // elevon and flaperon mixing are mutually exclusive
        if ( mix_elevon ) {
            actuator_norm[0] = mix_Gea * aileron_cmd + mix_Gee * elevator_cmd;
            actuator_norm[1] = mix_Gea * aileron_cmd - mix_Gee * elevator_cmd;
        } else if ( mix_flaperon ) {
            actuator_norm[0] = mix_Gfa * aileron_cmd + mix_Gff * flap_cmd;
            actuator_norm[5] = mix_Gfa * aileron_cmd - mix_Gff * flap_cmd;
        }
        // vtail mixing can't work with elevon mixing
        if ( mix_vtail && !mix_elevon) {
            actuator_norm[1] = mix_Gve * elevator_cmd + mix_Gvr * rudder_cmd;
            actuator_norm[3] = mix_Gve * elevator_cmd - mix_Gvr * rudder_cmd;
        }
        if ( mix_diff_thrust ) {
            actuator_norm[2] = mix_Gtt * throttle_cmd + mix_Gtr * rudder_cmd;
            actuator_norm[4] = mix_Gtt * throttle_cmd - mix_Gtr * rudder_cmd;
        }
    }
    
    // compute raw actuator output values from the normalized values
    norm2raw( actuator_norm, actuator_raw );
}


// read the receiver if new data is available.  If manual mode requested, immediate do the mixing and send the result to the servos
int receiver_process() {
    // New radio frame?
    if ( APM_RC.GetState() == 1 ) {
        // read channel data
	for ( int i = 0; i < NUM_CHANNELS; i++ ) {
            // save raw pulse value
            receiver_raw[i] = APM_RC.InputCh(i);
	}
 
        if ( receiver_raw[CH_8] > 1500 ) {
            // manual pass through requested, let's get it done right now
            raw2norm( receiver_raw, receiver_norm );
            sas_update( receiver_norm );
            mixing_update( receiver_norm, true /* do ch1-7 */, false /* do ch8 */ );
            actuator_update();
        }
        return 1;
    }
    return 0;
}


// set default raw actuator values
void actuator_set_defaults() {
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        if ( symmetrical[i] ) {
            // i.e. aileron, rudder, elevator
	    actuator_raw[i] = PWM_CENTER;
        } else {
            // i.e. throttle, flaps
    	    actuator_raw[i] = PWM_MIN;
        }
    }
}


// write the raw actuator values to the RC system
int actuator_update() {
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        APM_RC.OutputCh(i, actuator_raw[i] );
    }

    return 0;
}

