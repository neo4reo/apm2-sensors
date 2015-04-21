// Module to handle actuator input/output and mixing.

#include "config.h"

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

// Mix mode commands (format is cmd(byte), gain 1,, gain 2
#define MIX_DEFAULTS 0
#define MIX_AUTOCOORDINATE 1
#define MIX_THROTTLE_TRIM 2
#define MIX_FLAP_TRIM 3
#define MIX_ELEVONS 4
#define MIX_FLAPERONS 5
#define MIX_VTAIL 6
#define MIX_DIFF_THRUST 7


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
    config.sas_rollaxis = false;
    config.sas_pitchaxis = false;
    config.sas_yawaxis = false;
    config.sas_ch7tune = false;

    config.sas_rollgain = 0.0;
    config.sas_pitchgain = 0.0;
    config.sas_yawgain = 0.0;
    config.sas_ch7gain = 2.0;
};


// reset mixing parameters to startup defaults
void mixing_defaults() {
    config.mix_autocoord = false;
    config.mix_throttle_trim = false;
    config.mix_flap_trim = false;
    config.mix_elevon = false;
    config.mix_flaperon = false;
    config.mix_vtail = false;
    config.mix_diff_thrust = false;

    config.mix_Gac = 0.5; // aileron gain for autocoordination
    config.mix_Get = -0.1; // elevator trim w/ throttle gain
    config.mix_Gef = 0.1; // elevator trim w/ flap gain

    config.mix_Gea = 1.0; // aileron gain for elevons
    config.mix_Gee = 1.0; // elevator gain for elevons
    config.mix_Gfa = 1.0; // aileron gain for flaperons
    config.mix_Gff = 1.0; // flaps gain for flaperons
    config.mix_Gve = 1.0; // elevator gain for vtail
    config.mix_Gvr = 1.0; // rudder gain for vtail
    config.mix_Gtt = 1.0; // throttle gain for diff thrust
    config.mix_Gtr = 0.1; // rudder gain for diff thrust
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
        config.sas_rollaxis = enable;
        config.sas_rollgain = gain;
    } else if ( buf[0] == SAS_PITCHAXIS ) {
        config.sas_pitchaxis = enable;
        config.sas_pitchgain = gain;
    } else if ( buf[0] == SAS_YAWAXIS ) {
        config.sas_yawaxis = enable;
        config.sas_yawgain = gain;
    } else if ( buf[0] == SAS_CH7_TUNE ) {
        config.sas_ch7tune = enable;
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
        config.mix_autocoord = enable;
        config.mix_Gac = g1;
    } else if ( buf[0] == MIX_THROTTLE_TRIM ) {
        config.mix_throttle_trim = enable;
        config.mix_Get = g1;
    } else if ( buf[0] == MIX_FLAP_TRIM ) {
        config.mix_flap_trim = enable;
        config.mix_Gef = g1;
    } else if ( buf[0] == MIX_ELEVONS ) {
        config.mix_elevon = enable;
        config.mix_Gea = g1;
        config.mix_Gee = g2;
    } else if ( buf[0] == MIX_FLAPERONS ) {
        config.mix_flaperon = enable;
        config.mix_Gfa = g1;
        config.mix_Gff = g2;
    } else if ( buf[0] == MIX_VTAIL ) {
        config.mix_vtail = enable;
        config.mix_Gve = g1;
        config.mix_Gvr = g2;
    } else if ( buf[0] == MIX_DIFF_THRUST ) {
        config.mix_diff_thrust = enable;
        config.mix_Gtt = g1;
        config.mix_Gtr = g2;
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
        if ( symmetrical[i] || (i == 5 && config.mix_flaperon) ) {
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
    if ( config.sas_ch7tune ) {
        tune = config.sas_ch7gain * receiver_norm[6];
        if ( tune < 0.0 ) {
            tune = 0.0;
        } else if ( tune > 2.0 ) {
            tune = 2.0;
        }
    } 
    if ( config.sas_rollaxis ) {
        control_norm[0] += tune * config.sas_rollgain * imu_sensors[0];
    }
    if ( config.sas_pitchaxis ) {
        control_norm[1] += tune * config.sas_pitchgain * imu_sensors[1];
    }
    if ( config.sas_yawaxis ) {
        control_norm[3] += tune * config.sas_yawgain * imu_sensors[2];
    }
}

// compute the actuator (servo) values for each channel.  Handle all the requested mixing modes here.
void mixing_update( float control_norm[NUM_CHANNELS], bool do_ch1_6, bool do_ch7, bool do_ch8 ) {
    if ( do_ch1_6 ) {
        aileron_cmd = control_norm[0];
        elevator_cmd = control_norm[1];
        throttle_cmd = control_norm[2];
        rudder_cmd = control_norm[3];
        gear_cmd = control_norm[4];
        flap_cmd = control_norm[5];
    }
    if ( do_ch7 ) {
        ch7_cmd = control_norm[6];
    }
    if ( do_ch8 ) {
        ch8_cmd = control_norm[7];
    }
        
    // mixing modes that work at the 'command' level (before actuator value assignment)
    if ( config.mix_autocoord ) {
        rudder_cmd += config.mix_Gac * aileron_cmd;
    }
    if ( config.mix_throttle_trim ) {
        elevator_cmd += config.mix_Get * throttle_cmd;
    }
    if ( config.mix_flap_trim ) {
        elevator_cmd += config.mix_Gef * flap_cmd;
    }
  
    // copy default assignments as if no mixing
    if ( do_ch1_6 ) {
        actuator_norm[0] = aileron_cmd;
        actuator_norm[1] = elevator_cmd;
        actuator_norm[2] = throttle_cmd;
        actuator_norm[3] = rudder_cmd;
        actuator_norm[4] = gear_cmd;
        actuator_norm[5] = flap_cmd;
    }
    if ( do_ch7 ) {
        actuator_norm[6] = ch7_cmd;
    }
    if ( do_ch8 ) {
        actuator_norm[7] = ch8_cmd;
    }
    
    if ( do_ch1_6 ) {
        // elevon and flaperon mixing are mutually exclusive
        if ( config.mix_elevon ) {
            actuator_norm[0] = config.mix_Gea * aileron_cmd + config.mix_Gee * elevator_cmd;
            actuator_norm[1] = config.mix_Gea * aileron_cmd - config.mix_Gee * elevator_cmd;
        } else if ( config.mix_flaperon ) {
            actuator_norm[0] = config.mix_Gfa * aileron_cmd + config.mix_Gff * flap_cmd;
            actuator_norm[5] = config.mix_Gfa * aileron_cmd - config.mix_Gff * flap_cmd;
        }
        // vtail mixing can't work with elevon mixing
        if ( config.mix_vtail && !config.mix_elevon) {
            actuator_norm[1] = config.mix_Gve * elevator_cmd + config.mix_Gvr * rudder_cmd;
            actuator_norm[3] = config.mix_Gve * elevator_cmd - config.mix_Gvr * rudder_cmd;
        }
        if ( config.mix_diff_thrust ) {
            actuator_norm[2] = config.mix_Gtt * throttle_cmd + config.mix_Gtr * rudder_cmd;
            actuator_norm[4] = config.mix_Gtt * throttle_cmd - config.mix_Gtr * rudder_cmd;
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
 
        raw2norm( receiver_raw, receiver_norm );
        
        if ( receiver_raw[CH_8] > 1500 ) {
            // manual pass through requested, let's get it done right now
            sas_update( receiver_norm );
            mixing_update( receiver_norm, true /* ch1-6 */, true /* ch7 */, false /* no ch8 */ );
            actuator_update();
        } else {
            // autopilot mode, but let's update the sas gain tuning channel if requested
            mixing_update( receiver_norm, false /* ch1-6 */, config.sas_ch7gain /* ch7 */, false /* no ch8 */ );
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

