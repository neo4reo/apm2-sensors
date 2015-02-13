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
  // mixing modes that work at the 'command' level
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
  actuator_norm[0] = aileron_cmd;
  actuator_norm[1] = elevator_cmd;
  actuator_norm[2] = throttle_cmd;
  actuator_norm[3] = rudder_cmd;
  actuator_norm[4] = gear_cmd;
  actuator_norm[5] = flap_cmd;
  actuator_norm[6] = receiver_norm[6];
  actuator_norm[7] = receiver_norm[7];

  // elevon and flaperon mixing are mutually exclusive
  if ( mix_elevon ) {
    actuator_norm[0] = mix_Ga * aileron_cmd + mix_Ge * elevator_cmd;
    actuator_norm[1] = mix_Ga * aileron_cmd - mix_Ge * elevator_cmd;
  } else if ( mix_flaperon ) {
    actuator_norm[0] = mix_Ga * aileron_cmd + mix_Gf * flap_cmd;
    actuator_norm[6] = mix_Ga * aileron_cmd - mix_Gf * flap_cmd;
  }
  // vtail mixing can't work with elevon mixing
  if ( mix_vtail && !mix_elevon) {
    actuator_norm[1] = mix_Ge * elevator_cmd + mix_Gr * rudder_cmd;
    actuator_norm[3] = mix_Ge * elevator_cmd - mix_Gr * rudder_cmd;
  }
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
	        receiver_norm[i] = (float)(receiver_raw[i] - PWM_CENTER) / PWM_HALF_RANGE;
            } else {
	        // i.e. throttle, flaps
	        receiver_norm[i] = (float)(receiver_raw[i] - PWM_MIN) / (int32_t)PWM_RANGE;
            }
	}
        aileron_cmd = receiver_norm[0];
        elevator_cmd = receiver_norm[1];
        throttle_cmd = receiver_norm[2];
        rudder_cmd = receiver_norm[3];
        flap_cmd = receiver_norm[5];
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
        APM_RC.OutputCh(CH_8, actuator_raw[CH_8] );
    } else {
        // autopilot control
        for ( int i = 0; i < NUM_CHANNELS; i++ ) {
            APM_RC.OutputCh(i, actuator_raw[i] );
        }
    }


    return 0;
}

