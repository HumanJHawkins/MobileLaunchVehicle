public enum Speed {
    SPEED_ZERO(0, 0, "Zero."),
    SPEED_ONE(1, 0.20, "One.") {
        @Override
        // Disallow shifting to zero. (Can set to zero directly, but not by shifting.)
        public Speed previous() {
            return this;
        }

        ;
    },
    SPEED_TWO(2, 0.40, "Two."),
    SPEED_THREE(3, 0.60, "Three."),
    SPEED_FOUR(4, 0.80, "Four."),
    SPEED_FIVE(5, 1.00, "Five.") {
        @Override
        public Speed next() {
            return this;
        }

        ;
    };

    private final int speed;
    private final double powerFactor;
    private final String description;

    Speed(int speed, double powerFactor, String description) {
        this.speed = speed;
        this.powerFactor = powerFactor;
        this.description = description;
    }

    public Speed next() {
        // No bounds checking required here, because the last instance overrides
        return values()[ordinal() + 1];
    }

    public Speed previous() {
        // No bounds checking required here, because the last instance overrides
        return values()[ordinal() - 1];
    }

    private double speed() {
        return speed;
    }

    private double powerFactor() {
        return powerFactor;
    }

    private String description() {
        return description;
    }
}