package frc.robot.lib.motors.positionController;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class PositionIOSparkFlex implements PositionControllerIO{
    private final SparkFlex motor;
    private final double pivotOffset;

    private final SparkAbsoluteEncoder motorEncoder;

    public PositionIOSparkFlex(int deviceId, SparkMaxConfig motorConfig, double pivotOffset) {
        this.pivotOffset = pivotOffset;
        motor = new SparkFlex(deviceId, MotorType.kBrushless);

        motorEncoder = motor.getAbsoluteEncoder();
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(PositionControllerIOInputs inputs) {
        inputs.motorAppliedVolts = motor.getAppliedOutput();
        inputs.motorCurrentAmps = new double[] {motor.getOutputCurrent()};
        inputs.motorEncoder = getEncoder();
        inputs.motorTemp = motor.getMotorTemperature();
    }

    @Override
    public void setMotorVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        motor.setVoltage(volts);
    }

    public double getEncoder() {
        // testNumber = (testNumber > 100) ? 0 : (testNumber + .01);
        return (motorEncoder.getPosition() * 360) + pivotOffset;
        //return testNumber;
    }

    public double getVelocity() {
        return motorEncoder.getVelocity();
    }
}
