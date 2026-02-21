package frc.robot.lib.motors.positionController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionController extends SubsystemBase{
    private PositionControllerIO io;
    private final PositionControllerIOInputsAutoLogged inputs = new PositionControllerIOInputsAutoLogged();
    private boolean enabled;
    private String subsystem;

    public PositionController(PositionControllerIO io, String subsystem) {
        this.io = io;
        this.subsystem = subsystem;
    
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(subsystem + "Pivot", inputs);
    }

    public void setVoltage(double volts) {
        io.setMotorVoltage(volts);
    }

    public double getAngle() {
        return io.getEncoder();
    }

    public double getVelocity() {
        return io.getVelocity();
    }

    public void setMotorPosition(double rotations) {
        io.setMotorPosition(rotations);
    }

    public double getMotorSetpoint() {
        return io.getMotorSetpoint();
    }
}
