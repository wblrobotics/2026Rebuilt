package frc.robot.lib.motors.velocityController;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class VelocityIOSim implements VelocityControllerIO {
    private DCMotor maxGearbox = DCMotor.getNEO(1);
    private SparkMax sparkMax = new SparkMax(1, MotorType.kBrushless);
    
    private SparkMaxSim motorSim = new SparkMaxSim(sparkMax, maxGearbox);

    private double motorAppliedVolts = 0.0;

    @Override
    public void updateInputs(VelocityControllerIOInputs inputs) {    
        inputs.motorAppliedVolts = motorAppliedVolts;
        inputs.motorCurrentAmps = new double[] {motorSim.getMotorCurrent()};
        inputs.motorVelocityRadsPerSec = getVelocity();
    }

    public void simulationPeriodic() {
        motorSim.iterate(getVelocity(), getVBus(), 0.02);
    }

    public void periodic() {
        motorSim.iterate(getVelocity(), getVBus(), 0.02);
    }

    private double getVelocity() {
        // motorSim.getVelocity returns RPM, convert to radians per second
        return motorSim.getVelocity() * ((2 * Math.PI) / 60);
    }

    public double getVBus() {
        return motorSim.getBusVoltage();
    }

    @Override
    public void setMotorVoltage(double volts) {
        motorAppliedVolts = MathUtil.clamp(volts, -12, 12);
        motorSim.setBusVoltage(motorAppliedVolts);
    }
}
