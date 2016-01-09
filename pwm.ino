#define PWM_CENTER 1520
#define PWM_HALF_RANGE 413
#define PWM_RANGE (PWM_HALF_RANGE * 2)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)
#define PWM_MAX (PWM_CENTER + PWM_HALF_RANGE)


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


// compute raw pwm values from normalized command values.
// (handle actuator reversing here.)
void norm2raw( float norm[NUM_CHANNELS], uint16_t raw[NUM_CHANNELS] ) {
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        // convert to pulse length (special case ch6 when in flaperon mode)
        if ( symmetrical[i] || (i == 5 && config.mix_flaperon) ) {
            // i.e. aileron, rudder, elevator
            //Serial.println(i);
            //Serial.println(config.act_rev[i]);
	    raw[i] = PWM_CENTER + (int)(PWM_HALF_RANGE * norm[i] * config.act_gain[i]);
        } else {
	    // i.e. throttle, flaps
            if ( config.act_gain[i] > 0.0 ) {
	        raw[i] = PWM_MIN + (int)(PWM_RANGE * norm[i] * config.act_gain[i]);
            } else {
	        raw[i] = PWM_MAX + (int)(PWM_RANGE * norm[i] * config.act_gain[i]);
            }
        }
        if ( raw[i] < PWM_MIN ) {
            raw[i] = PWM_MIN;
        }
        if ( raw[i] > PWM_MAX ) {
            raw[i] = PWM_MAX;
        }
    }
}


// read the receiver if new data is available.  If manual mode requested, immediate do the mixing and send the result to the servos
int PWM_process() {
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


