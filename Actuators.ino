#define PWM_CENTER 1500
#define PWM_HALF_RANGE 500
#define PWM_RANGE (PWM_HALF_RANGE * 2)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)
#define PWM_MAX (PWM_CENTER + PWM_HALF_RANGE)


bool mix_enable = false;
float mix_gains[NUM_CHANNELS][NUM_CHANNELS];

// define if a channel is symmetrical or not (i.e. mapped to [0,1] for throttle, flaps, spoilers; [-1,1] for aileron, elevator, rudder
bool symmetrical[NUM_CHANNELS] = {1, 1, 0, 1, 0, 0, 0, 0};


void mixing_init() {
    /* default to straight input -> output channel pass through */
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        mix_gains[i][i] = 1.0;
    }
};


int receiver_read() {
    // New radio frame?
    if ( APM_RC.GetState() == 1 ) {
        Serial.println("new receiver data");
        // read channel data
	for ( int i = 0; i < NUM_CHANNELS; i++ ) {
            // save raw pulse value
            receiver_raw[i] = APM_RC.InputCh(i);
            // convert to normalized form
            if ( symmetrical[i] ) {
                // i.e. aileron, rudder, elevator
	        receiver_norm[i] = (receiver_raw[i] - PWM_CENTER) / PWM_HALF_RANGE;
            } else {
	        // i.e. throttle, flaps
	        receiver_norm[i] = (receiver_raw[i] - PWM_MIN) / PWM_RANGE;
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

