package frc.robot.lib.motors.motorController;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

public class MotorIOSparkMax implements MotorControllerIO{
    private final SparkMax motor;

    private final SparkAbsoluteEncoder motorEncoder;

    private Boolean thresholdMet = false;
    private double currentThreshold;

    public MotorIOSparkMax(int deviceId, SparkMaxConfig motorConfig, double currentThreshold) {
        this.currentThreshold = currentThreshold;

        motor = new SparkMax(deviceId, MotorType.kBrushless);

        motorEncoder = motor.getAbsoluteEncoder();
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(MotorControllerIOInputs inputs) {
        inputs.motorAppliedVolts = motor.getAppliedOutput();
        inputs.motorCurrentAmps = new double[] {motor.getOutputCurrent()};
        inputs.motorEncoder = getEncoder();
        inputs.motorTemp = motor.getMotorTemperature();
    }

    @Override
    public void setMotorVoltage(double volts) {
        volts 
        
        = MathUtil.clamp(volts, -12, 12);
        motor.setVoltage(volts);
    }

    public void setMotor(double percent) {
        percent = MathUtil.clamp(percent, -1, 1);
        motor.set(percent);
    }

    public double getEncoder() {
        return motorEncoder.getPosition();
    }

    public boolean assessCurrent() {
        if (motor.getOutputCurrent() > currentThreshold) {
            thresholdMet = true;
        } else {
            thresholdMet = false;
        }

        return thresholdMet;
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }
}
