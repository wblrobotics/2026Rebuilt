package frc.robot.lib.motors.motorController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorController extends SubsystemBase{
    private MotorControllerIO io;
    private final MotorControllerIOInputsAutoLogged inputs = new MotorControllerIOInputsAutoLogged();
    private String subsystem;
    private String number;

    public MotorController(MotorControllerIO io, String subsystem, String number) {
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
        io.setMotor(percent);
    }

    public boolean hasCurrentReached() {
       return io.assessCurrent();
    }

    public double getCurrent() {
        return io.getCurrent();
    }

    public double getEncoder() {
        return io.getEncoder();
    }
}
