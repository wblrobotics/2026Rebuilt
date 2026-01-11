package frc.robot.lib.motors.velocityController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

public class VelocityIOSparkFlex implements VelocityControllerIO{
    private final SparkFlex motor;

    private final RelativeEncoder motorEncoder;

    public VelocityIOSparkFlex(int deviceId, SparkMaxConfig motorConfig) {
        motor = new SparkFlex(deviceId, MotorType.kBrushless);

        motorEncoder = motor.getEncoder();
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(VelocityControllerIOInputs inputs) {
        inputs.motorAppliedVolts = motor.getAppliedOutput();
        inputs.motorCurrentAmps = new double[] {motor.getOutputCurrent()};
        inputs.motorVelocityRadsPerSec = getVelocity();
    }

    @Override
    public void setMotorVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        motor.setVoltage(volts);
    }

    /** Requires a decimal percentage */
    public void setVelocity(double velocity) {
        velocity = MathUtil.clamp(velocity, -1, 1);
        motor.set(velocity);
    }

    private double getVelocity() {
        // motorEncoder.getVelocity() returns RPM, convert to radians per second
        return motorEncoder.getVelocity() * ((2 * Math.PI) / 60);
    }
}
