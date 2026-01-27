package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.FieldConstants;
import frc.robot.lib.motors.velocityController.VelocityController;
import frc.robot.lib.motors.velocityController.VelocityIOSparkMax;
import frc.robot.lib.swerve.updated.SwerveDrive;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Outtake extends SubsystemBase {
    private VelocityController motorOne;
    private VelocityController motorTwo;

    private SwerveDrive swerve;
    private Boolean hopperEmpty = false;

    private InterpolatingDoubleTreeMap launchMap = new InterpolatingDoubleTreeMap();

    private final int motorOneId = Constants.OuttakeConstants.motorOneId;
    private final int motorTwoId = Constants.OuttakeConstants.motorTwoId;

    public Outtake(String robotType, SwerveDrive drive) {
        this.swerve = drive;
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(30);
        config.closedLoop.feedForward // Set Feedforward gains for the velocity controller
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

        // Set the prelearned distances (inches) with respective velocities (RPM)
        launchMap.put(20.0, 200.0);
        launchMap.put(40.0, 400.0);
        launchMap.put(60.0, 600.0);
        launchMap.put(80.0, 800.0);
        launchMap.put(100.0, 1000.0);
        launchMap.put(120.0, 1200.0);
        launchMap.put(140.0, 1400.0);
        launchMap.put(160.0, 1600.0);
        launchMap.put(180.0, 1800.0);
        launchMap.put(200.0, 2000.0);
    }

    public void periodic() {
        motorOne.updateInputs();
        motorTwo.updateInputs();
    }

    public void setVoltage(double volts) {
        motorOne.setVoltage(volts);
        motorTwo.setVoltage(volts);
    }

    public Command quickLaunch() {
        double motorOneSpeed = 1000;
        double motorTwoSpeed = 1000;

        return Commands.sequence(
                runOnce(() -> {
                    motorOne.setSpeed(motorOneSpeed);
                    motorTwo.setSpeed(motorTwoSpeed);
                }),
                Commands.waitSeconds(1),
                runOnce(() -> {
                    stop();
                }));
    }

    // Runs the launcher at variable RPM in relation to distance from the hub. Motors stop when the hopper is empty
    public Command fullLaunch() {
        return Commands.sequence(
                run(() -> {
                    double velocity = getVelocityTarget(checkDistance(FieldConstants.Elements.hubPose));

                    motorOne.setSpeed(velocity);
                    motorTwo.setSpeed(velocity);
                }),
                Commands.waitUntil(() -> hopperEmpty),
                runOnce(() -> {
                    stop();
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

    public double getVelocityTarget(double distance) {
        return launchMap.get(distance);
    }

    /** Checks the distance from the bot to the target */
    public double checkDistance(Pose2d target) {
        return Math.sqrt(
                Math.pow(swerve.targetOffset(target).getX(), 2) + Math.pow(swerve.targetOffset(target).getY(), 2));
    }

}
