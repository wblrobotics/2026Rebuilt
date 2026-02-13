package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.FieldConstants;
import frc.robot.current.Constants.OuttakeConstants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.lib.motors.velocityController.VelocityController;
import frc.robot.lib.motors.velocityController.VelocityIOSparkFlex;
import frc.robot.lib.motors.velocityController.VelocityIOSparkMax;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Outtake extends SubsystemBase {
    private VelocityController highMotor;
    private VelocityController lowMotor;

    private Drive swerve;
    private Boolean hopperEmpty = false;

    private InterpolatingDoubleTreeMap launchMap = new InterpolatingDoubleTreeMap();

    private final int highMotorId = OuttakeConstants.motorOneId;
    private final int lowMotorId = OuttakeConstants.motorTwoId;

    public Outtake(Drive drive) {
        this.swerve = drive;
        SparkFlexConfig lowConfig = new SparkFlexConfig();
        SparkFlexConfig highConfig = new SparkFlexConfig();
        lowConfig.inverted(false);
        highConfig.inverted(true);
        lowConfig.smartCurrentLimit(70);
        highConfig.smartCurrentLimit(70);
        lowConfig.closedLoop
                .p(OuttakeConstants.kP)
                .i(OuttakeConstants.kI)
                .d(OuttakeConstants.kD)
            .feedForward // Set Feedforward gains for the velocity controller
                .kS(OuttakeConstants.kS) // Static gain (volts)
                .kV(OuttakeConstants.kV) // Velocity gain (volts per RPM)
                .kA(OuttakeConstants.kA); // Acceleration gain (volts per RPM/s)
        highConfig.closedLoop
                .p(OuttakeConstants.kP)
                .i(OuttakeConstants.kI)
                .d(OuttakeConstants.kD)
            .feedForward // Set Feedforward gains for the velocity controller
                .kS(OuttakeConstants.kS) // Static gain (volts)
                .kV(OuttakeConstants.kV) // Velocity gain (volts per RPM)
                .kA(OuttakeConstants.kA); // Acceleration gain (volts per RPM/s)

        
        switch (Constants.robot) {
            case "Real":
                highMotor = new VelocityController(new VelocityIOSparkFlex(highMotorId, highConfig), "Outtake", "1");
                lowMotor = new VelocityController(new VelocityIOSparkFlex(lowMotorId, lowConfig), "Outtake", "2");

                break;
            case "SIM":
                // Just don't use sim.

                break;
            default:
                highMotor = new VelocityController(new VelocityIOSparkFlex(highMotorId, highConfig), "Outtake", "1");
                lowMotor = new VelocityController(new VelocityIOSparkFlex(lowMotorId, lowConfig), "Outtake", "2");
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
        highMotor.updateInputs();
        lowMotor.updateInputs();
    }

    public void setVoltage(double volts) {
        highMotor.setVoltage(volts);
        lowMotor.setVoltage(volts);
    }

    public Command quickLaunch() {
        double motorOneSpeed = OuttakeConstants.velocityDefault;
        double motorTwoSpeed = OuttakeConstants.velocityDefault;

        return Commands.sequence(
                runOnce(() -> {
                    highMotor.setSpeed(motorOneSpeed);
                    lowMotor.setSpeed(motorTwoSpeed);
                }),
                Commands.waitSeconds(1),
                runOnce(() -> {
                    stop();
                }));
    }

    public Command continuousLaunch(){
        double motorOneSpeed = OuttakeConstants.velocityDefault;
        double motorTwoSpeed = OuttakeConstants.velocityDefault;

        return Commands.sequence(
                run(() -> {
                    highMotor.setSpeed(motorOneSpeed);
                    lowMotor.setSpeed(motorTwoSpeed);
                }));
    }

    // Runs the launcher at variable RPM in relation to distance from the hub.
    // Motors stop when the hopper is empty
    public Command fullLaunch() {
        return Commands.sequence(
                run(() -> {
                    double velocity = getVelocityTarget(checkDistance(FieldConstants.Elements.hubPose));

                    highMotor.setSpeed(velocity);
                    lowMotor.setSpeed(velocity);
                }),
                Commands.waitUntil(() -> hopperEmpty),
                runOnce(() -> {
                    stop();
                }));
    }

    /** Stops all the motors */
    public Command stop() {
        return Commands.run(() -> {
            highMotor.stop();
            lowMotor.stop();
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
