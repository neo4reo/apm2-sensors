#define PWM_CENTER 1500
#define PWM_HALF_RANGE 450
#define PWM_RANGE (PWM_HALF_RANGE * 2)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)
#define PWM_MAX (PWM_CENTER + PWM_HALF_RANGE)

// Mix mode commands (format is cmd(byte), gain 1 (float), gain 2 (float)
#define MIX_DEFAULTS 0
#define MIX_AUTOCOORDINATE 1
#define MIX_THROTTLE_TRIM 2
#define MIX_FLAP_TRIM 3
#define MIX_ELEVONS 4
#define MIX_FLAPERONS 5
#define MIX_VTAIL 6

bool mix_autocoord = false;
bool mix_throttle_trim = false;
bool mix_flap_trim = false;
bool mix_elevon = false;
bool mix_flaperon = false;
bool mix_vtail = false;

float mix_Gac = 0.5; // aileron gain for autocoordination
float mix_Get = -0.1; // elevator trim w/ throttle gain
float mix_Gef = 0.1; // elevator trim w/ flap gain

float mix_Ga = 1.0; // aileron gain for elevons or flaperons
float mix_Ge = 1.0; // elevator gain for elevons or vtail
float mix_Gf = 1.0; // flaps gain for flaperons
float mix_Gr = 1.0; // rudder gain for vtail

// define if a channel is symmetrical or not (i.e. mapped to [0,1] for throttle, flaps, spoilers; [-1,1] for aileron, elevator, rudder
bool symmetrical[NUM_CHANNELS] = {1, 1, 0, 1, 0, 0, 0, 0};


// reset mixing parameters to startup defaults
void mixing_defaults() {
    mix_autocoord = false;
    mix_throttle_trim = false;
    mix_flap_trim = false;
    mix_elevon = false;
    mix_flaperon = false;
    mix_vtail = false;

    mix_Gac = 0.5; // aileron gain for autocoordination
    mix_Get = -0.1; // elevator trim w/ throttle gain
    mix_Gef = 0.1; // elevator trim w/ flap gain

    mix_Ga = 1.0; // aileron gain for elevons or flaperons
    mix_Ge = 1.0; // elevator gain for elevons or vtail
    mix_Gf = 1.0; // flaps gain for flaperons
    mix_Gr = 1.0; // rudder gain for vtail
};


int mixing_command_parse(byte *buf) {
  float g1 = *(int16_t *)(buf[1]) / 32767;
  float g2 = *(int16_t *)(buf[3]) / 32767;
  if ( buf[0] == MIX_DEFAULTS ) {
    mixing_defaults();
  } else if ( buf[0] == MIX_AUTOCOORDINATE ) {
    mix_autocoord = true;
    mix_Gac = g1;
  } else if ( buf[0] == MIX_THROTTLE_TRIM ) {
    mix_throttle_trim = true;
    mix_Get = g1;
  } else if ( buf[0] == MIX_FLAP_TRIM ) {
    mix_flap_trim = true;
    mix_Gef = g1;
  } else if ( buf[0] == MIX_ELEVONS ) {
    mix_elevon = true;
    mix_Ga = g1;
    mix_Ge = g2;
  } else if ( buf[0] == MIX_FLAPERONS ) {
    mix_flaperon = true;
    mix_Ga = g1;
    mix_Gf = g2;
  } else if ( buf[0] == MIX_VTAIL ) {
    mix_vtail = true;
    mix_Ge = g1;
    mix_Gr = g2;
  }
}


void mixing_update() {
    //for ( int j = 0; j < NUM_CHANNELS; j++ ) {
    //    actuator_norm[j] = 0.0;
    //    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
    //        actuator_norm[j] += mix_gains[ireceiver_norm[i]
    //    }
    //}
}

int receiver_read() {
    // New radio frame?
    if ( APM_RC.GetState() == 1 ) {
        // read channel data
	for ( int i = 0; i < NUM_CHANNELS; i++ ) {
            // save raw pulse value
            receiver_raw[i] = APM_RC.InputCh(i);
            // convert to normalized form
            if ( symmetrical[i] ) {
                // i.e. aileron, rudder, elevator
	        receiver_norm[i] = ((int32_t)receiver_raw[i] - PWM_CENTER) * 10000 / PWM_HALF_RANGE;
            } else {
	        // i.e. throttle, flaps
	        receiver_norm[i] = ((int32_t)receiver_raw[i] - PWM_MIN) * 10000 / (int32_t)PWM_RANGE;
            }
	}
        return 1;
    }
    return 0;
}


int actuators_update() {
    if ( receiver_raw[CH_8] > 1500 ) {
        // manual pass through
        for ( int i = 0; i < NUM_CHANNELS - 1; i++ ) {
            APM_RC.OutputCh(i, receiver_raw[i] );
        }
        APM_RC.OutputCh(CH_8, actuator_pos[CH_8] );
    } else {
        // autopilot control
        for ( int i = 0; i < NUM_CHANNELS; i++ ) {
            APM_RC.OutputCh(i, actuator_pos[i] );
        }
    }


    return 0;
}

