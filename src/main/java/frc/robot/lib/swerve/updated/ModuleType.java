package frc.robot.lib.swerve.updated;

public enum ModuleType {
    SDSMK4iL1,
    SDSMK4iL2,
    SDSMK4iL3;

    /**
     * Max speed in ft/s. Implied that Neo V1.0/1.1 are used.
     */
    public double maxSpeed() {
        switch (this) {
            case SDSMK4iL1:
                return 12.5;
            case SDSMK4iL2:
                return 15.1;
            case SDSMK4iL3:
                return 16.6;
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
            default:
                return 150 / 7;
        }
    }

}
