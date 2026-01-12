package frc.robot.lib.motors.motorController;

import org.littletonrobotics.junction.AutoLog;

public interface MotorControllerIO {
    @AutoLog
    public static class MotorControllerIOInputs {
        public double motorAppliedVolts = 0.0;
        public double[] motorCurrentAmps = new double[] {};
        public double motorEncoder = 0.0;
        public double motorTemp = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(MotorControllerIOInputs inputs) {}

    /** Run the motor at the specified voltage. */
    public default void setMotorVoltage(double volts) {}

    public abstract void setMotor(double percent);

    public abstract double getEncoder();

    public abstract boolean assessCurrent();

    public abstract double getCurrent();
}
