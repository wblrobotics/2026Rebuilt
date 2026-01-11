package frc.robot.lib.swerve.updated;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class GyroIOADXRS450 implements GyroIO {
    private final ADXRS450_Gyro ADXRS450;
    
    public GyroIOADXRS450() {
        System.out.println("[Init] Creating GyroIOADXRS450");

        ADXRS450 = new ADXRS450_Gyro();
        ADXRS450.reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
        double angle = -ADXRS450.getAngle();
        double rate = -ADXRS450.getRate();
        inputs.connected = ADXRS450.isConnected();
        inputs.rollPositionRad = 0.0;
        inputs.pitchPositionRad = 0.0;
        inputs.yawPositionRad = Units.degreesToRadians(angle);
        inputs.rollVelocityRadPerSec = 0.0;
        inputs.pitchVelocityRadPerSec = 0.0;
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(rate);
    }

    public double getLeftRightAngle() { return ADXRS450.getAngle(); }
}
