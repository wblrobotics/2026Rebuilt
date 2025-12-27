package frc.robot.lib.swerve.updated;

public enum ModuleType {
    SDSMK4iL1(12.5, 8.14, 150/7),
    SDSMK4iL2(15.1, 6.75, 150 / 7),
    SDSMK4iL3(16.6, 6.12, 150 / 7),
    SDSMK5iR1(16.8, 7.03, 26),
    SDSMK5iR2(19.6, 6.03, 26),
    SDSMK5iR3(22.5, 5.27, 26);

    private final double maxSpeed;
    private final double driveReduction;
    private final double turnReduction;

    ModuleType(double maxSpeed, double driveReduction, double turnReduction) {
        this.maxSpeed = maxSpeed;
        this.driveReduction = driveReduction;
        this.turnReduction = turnReduction;
    }

    /**
     * Max speed in ft/s.
     */
    public double maxSpeed() {
        return this.maxSpeed;
    }

    public double driveReduction() {
        return this.driveReduction;
    }

    public double turnReduction() {
        return this.turnReduction;
    }

}
