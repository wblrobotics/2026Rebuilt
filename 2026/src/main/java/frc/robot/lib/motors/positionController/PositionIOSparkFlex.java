package frc.robot.lib.motors.positionController;

import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;

public class PositionIOSparkFlex implements PositionControllerIO{
    private final SparkFlex motor;
    private final double pivotOffset;

    private final SparkAbsoluteEncoder motorEncoder;
    private SparkClosedLoopController pidController;

    public PositionIOSparkFlex(int deviceId, SparkFlexConfig motorConfig, double pivotOffset) {
        this.pivotOffset = pivotOffset;
        motor = new SparkFlex(deviceId, MotorType.kBrushless);

        motorEncoder = motor.getAbsoluteEncoder();
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = motor.getClosedLoopController();
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

    public void setMotorPosition(double rotations) {
        pidController.setSetpoint(rotations, SparkBase.ControlType.kPosition);
    }
}
