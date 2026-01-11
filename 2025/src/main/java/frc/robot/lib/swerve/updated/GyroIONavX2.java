package frc.robot.lib.swerve.updated;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.util.Units;

public class GyroIONavX2 implements GyroIO{
    private final AHRS navX = new AHRS(NavXComType.kMXP_SPI);

    public GyroIONavX2() {
        System.out.println("[Init] Creating GyroIOnavX2");

        navX.reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
        double angle = -navX.getYaw();
        double rate = -navX.getRawGyroZ();
        inputs.connected = navX.isConnected();
        inputs.rollPositionRad = navX.getRoll();
        inputs.pitchPositionRad = navX.getPitch();
        inputs.yawPositionRad = Units.degreesToRadians(angle);
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(navX.getRawGyroX());
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(navX.getRawGyroY());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(rate);
    }

    public double getLeftRightAngle() { return navX.getRoll();}
}
