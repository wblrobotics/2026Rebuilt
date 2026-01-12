package frc.robot.lib.swerve.updated;

public class PIDConfig {
    private double kp;
    private double ki;
    private double kd;
    private double ks;
    private double kv;

    public PIDConfig(double kp, double ki, double kd, double ks, double kv) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.ks = ks;
        this.kv = kv;
    }

    public double getKp() {
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
    }

    public double getKs() {
        return ks;
    }

    public double getKv() {
        return kv;
    }
}
