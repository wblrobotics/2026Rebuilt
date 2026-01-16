package frc.robot.current.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.lib.motors.velocityController.VelocityController;
import frc.robot.lib.motors.velocityController.VelocityIOSparkMax;

public class Outtake extends SubsystemBase {
    private VelocityController motorOne;
    private VelocityController motorTwo;
    private VelocityController motorThree;

    private final int motorOneId = Constants.OuttakeConstants.motorOneId;
    private final int motorTwoId = Constants.OuttakeConstants.motorTwoId;
    private final int motorThreeId = Constants.OuttakeConstants.motorThreeId;

    public Outtake(String robotType) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(30);
        config.closedLoop.feedForward                       // Set PID gains for the velocity controller
            .kS(0)
            .kV(0);
            

        switch (robotType) {
            case "Real":
                motorOne = new VelocityController(new VelocityIOSparkMax(motorOneId, config), "Outtake", "1");
                motorTwo = new VelocityController(new VelocityIOSparkMax(motorTwoId, config), "Outtake", "2");
                motorThree = new VelocityController(new VelocityIOSparkMax(motorThreeId, config), "Outtake", "3");

                break;
            case "SIM":
                // Just don't use sim.

                break;
            default:
                motorOne = new VelocityController(new VelocityIOSparkMax(motorOneId, config), "Outtake", "1");
                motorTwo = new VelocityController(new VelocityIOSparkMax(motorTwoId, config), "Outtake", "2");
                motorThree = new VelocityController(new VelocityIOSparkMax(motorThreeId, config), "Outtake", "3");
                break;
        }
    }

    public void periodic() {
        motorOne.updateInputs();
        motorTwo.updateInputs();
        motorThree.updateInputs();

    }

    public void setVoltage(double volts) {
        motorOne.setVoltage(volts);
        motorTwo.setVoltage(volts);
        motorThree.setVoltage(volts);
    }

    public Command launch() {
        return Commands.sequence(
                runOnce(() -> {
                    motorOne.setVoltage(4);
                }),
                Commands.waitSeconds(.5),
                runOnce(() -> {
                    motorOne.setVoltage(0);
                }));
    }

    /** Stops all the motors */
    public Command stop() {
        return Commands.run(() -> {
            motorOne.setVoltage(0);
            motorTwo.setVoltage(0);
            motorThree.setVoltage(0);
        },
                this);
    }

}
