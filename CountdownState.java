public enum CountdownState {
    COUNTDOWN_HOLDING(0, "holding.") { // This position set in hardware.
        @Override
        public CountdownState previous() {
            return this;    // there is no previous for first position.
        };
    },
    COUNTDOWN_COUNTING(1, "counting."),
    COUNTDOWN_COMPLETE(2, "complete.") {
        @Override
        public CountdownState next() {
            return this;    // there is no next for last position up.
        };
    };

    private final int state;
    private final String description;

    CountdownState(int state, String description) {
        this.state = state;
        this.description = description;
    }

    public CountdownState next() {
        // No bounds checking required here, because the last instance overrides
        return values()[ordinal() + 1];
    }

    public CountdownState previous() {
        // No bounds checking required here, because the last instance overrides
        return values()[ordinal() - 1];
    }

    private double state() {
        return state;
    }

    private String description() {
        return description;
    }
}

