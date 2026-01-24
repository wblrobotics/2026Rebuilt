package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.lib.motors.velocityController.VelocityController;
import frc.robot.lib.motors.velocityController.VelocityIOSparkMax;
import frc.robot.lib.swerve.updated.SwerveDrive;

public class Outtake extends SubsystemBase {
    private VelocityController motorOne;
    private VelocityController motorTwo;

    private final int motorOneId = Constants.OuttakeConstants.motorOneId;
    private final int motorTwoId = Constants.OuttakeConstants.motorTwoId;

    public Outtake(String robotType, SwerveDrive drive) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(30);
        config.closedLoop.feedForward                       // Set Feedforward gains for the velocity controller
            .kS(Constants.OuttakeConstants.kS)
            .kV(Constants.OuttakeConstants.kV);
            

        switch (robotType) {
            case "Real":
                motorOne = new VelocityController(new VelocityIOSparkMax(motorOneId, config), "Outtake", "1");
                motorTwo = new VelocityController(new VelocityIOSparkMax(motorTwoId, config), "Outtake", "2");

                break;
            case "SIM":
                // Just don't use sim.

                break;
            default:
                motorOne = new VelocityController(new VelocityIOSparkMax(motorOneId, config), "Outtake", "1");
                motorTwo = new VelocityController(new VelocityIOSparkMax(motorTwoId, config), "Outtake", "2");
                break;
        }
    }

    public void periodic() {
        motorOne.updateInputs();
        motorTwo.updateInputs();
    }

    public void setVoltage(double volts) {
        motorOne.setVoltage(volts);
        motorTwo.setVoltage(volts);
    }

    public Command launch() {
        double motorOneSpeed = 4000;
        double motorTwoSpeed = 4100;

        return Commands.sequence(
                runOnce(() -> {
                    motorOne.setSpeed(motorOneSpeed);
                    motorTwo.setSpeed(motorTwoSpeed);
                }),
                Commands.waitSeconds(2),
                runOnce(() -> {
                    motorOne.setSpeed(0);
                    motorTwo.setSpeed(0);
                }));
    }

    /** Stops all the motors */
    public Command stop() {
        return Commands.run(() -> {
            motorOne.stop();
            motorTwo.stop();
        },
            this);
    }
}
