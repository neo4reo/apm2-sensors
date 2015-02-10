float mix_gains[NUM_CHANNELS][NUM_CHANNELS];

mixing_init() {
    /* default to straight input -> output channel pass through */
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        mix_gains[i][i] = 1.0;
    }
};
