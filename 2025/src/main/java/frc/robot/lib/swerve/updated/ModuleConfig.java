package frc.robot.lib.swerve.updated;

public class ModuleConfig {
    public int driveID;
    public int turnID;
    public int encoderID;
    public double encoderOffset;

    /**
     * 
     * @param driveID CAN ID of the drive motor
     * @param turnID CAN ID of the turn motor
     * @param encoderID CAN ID of the encoder
     * @param encoderOffset The angle offset from forward of the specific wheel in degrees. 
     */
    public ModuleConfig(int driveID, int turnID, int encoderID, double encoderOffset) {
        this.driveID = driveID;
        this.turnID = turnID;
        this.encoderID = encoderID;
        this.encoderOffset = encoderOffset;
    }

    public int getDriveID() {
        return driveID;
    }

    public int getTurnID() {
        return turnID;
    }

    public int getEncoderID() {
        return encoderID;
    }

    public double getEncoderOffset() {
        return Math.toRadians(encoderOffset);
    }
}
