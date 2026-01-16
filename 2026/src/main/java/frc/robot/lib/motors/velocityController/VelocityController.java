package frc.robot.lib.motors.velocityController;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VelocityController extends SubsystemBase{
    private VelocityControllerIO io;
    private final VelocityControllerIOInputsAutoLogged inputs = new VelocityControllerIOInputsAutoLogged();
    private String subsystem;
    private String number;

    public VelocityController(VelocityControllerIO io, String subsystem, String number) {
        this.io = io;
        this.subsystem = subsystem;
        this.number = number;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(subsystem + "Fly" + number, inputs);
    }

    public void setVoltage(double volts) {
        io.setMotorVoltage(volts);
    }

    public void setPercent(double percent) {
        io.setPercent(percent);
    }

    public double getVelocityRadians() {
        return inputs.motorVelocityRadsPerSec;
    }

    public double getVelocityRPM() {
        return inputs.motorVelocityRotationPerMinute;
    }

    public double[] getCurrent() {
        return inputs.motorCurrentAmps;
    }
}
