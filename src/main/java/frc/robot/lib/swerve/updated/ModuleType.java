package frc.robot.lib.swerve.updated;

public enum ModuleType {
    SDSMK4iL1,
    SDSMK4iL2,
    SDSMK4iL3,
    SDSMK5iR1,
    SDSMK5iR2,
    SDSMK5iR3;

    /**
     * Max speed in ft/s.
     */
    public double maxSpeed() {
        switch (this) {

            // Implied that NEO V1.0/1.1 are used
            case SDSMK4iL1:
                return 12.5;
            case SDSMK4iL2:
                return 15.1;
            case SDSMK4iL3:
                return 16.6;

            // Implied that NEO Vortex are used
            case SDSMK5iR1:
                return 16.8;
            case SDSMK5iR2:
                return 19.6;
            case SDSMK5iR3:
                return 22.5;
            default:
                return 1;
        }
    }

    public double driveReduction() {
        switch (this) {
            case SDSMK4iL1:
                return 8.14;
            case SDSMK4iL2:
                return 6.75;
            case SDSMK4iL3:
                return 6.12;

            case SDSMK5iR1:
                return 7.03;
            case SDSMK5iR2:
                return 6.03;
            case SDSMK5iR3:
                return 5.27;
            default:
                return 1;
        }
    }

    public double turnReduction() {
        switch (this) {
            case SDSMK4iL1:
                return 150 / 7;
            case SDSMK4iL2:
                return 150 / 7;
            case SDSMK4iL3:
                return 150 / 7;

            case SDSMK5iR1:
                return 26;
            case SDSMK5iR2:
                return 26;
            case SDSMK5iR3:
                return 26;
            default:
                return 150 / 7;
        }
    }

}
