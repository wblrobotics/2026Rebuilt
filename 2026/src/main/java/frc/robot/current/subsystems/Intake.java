package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.Constants.IntakeConstants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSparkFlex;
import frc.robot.lib.motors.motorController.MotorIOSparkMax;
import frc.robot.lib.motors.positionController.PIDConfig;
import frc.robot.lib.motors.positionController.PositionController;
import frc.robot.lib.motors.positionController.PositionIOSparkMax;

public class Intake extends SubsystemBase {
  private MotorController intakeMotor;
  private PositionController pivotMotor;

  private final int rightMotorID = Constants.IntakeConstants.motorID;
  private final int pivotMotorID = Constants.IntakeConstants.pivotMotorID;
  
  public Intake(String robotType, Drive drive) {
    // TODO: change table and entry keys
    // table = NetworkTableInstance.getDefault().getTable("default");
    // visionTarget = table.getEntry("default");

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted(true);
    rightConfig.smartCurrentLimit(30);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.inverted(false);
    pivotConfig.smartCurrentLimit(30);

    switch (robotType) {
      case "Real":
        intakeMotor = new MotorController(new MotorIOSparkFlex(rightMotorID, rightConfig, 35), "Intake", "1");
        pivotMotor = new PositionController(new PositionIOSparkMax(pivotMotorID, rightConfig, 0.0,
          new PIDConfig(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA)),
          "Intake");
        break;
      case "SIM":
        // Just don't use sim.

        break;
      default:
        intakeMotor = new MotorController(new MotorIOSparkMax(rightMotorID, rightConfig, 30), "Intake", "1");
        pivotMotor = new PositionController(new PositionIOSparkMax(pivotMotorID, rightConfig, 0.0,
          new PIDConfig(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA)),
          "Intake");
        break;
    }
  }

  public void periodic() {
    intakeMotor.updateInputs();
    pivotMotor.updateInputs();
  }

  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  public void setPosition(double setpoint){
    pivotMotor.setMotorPosition(setpoint);
  }

  public Command launch() {
    double percent = 20;

    return Commands.sequence(
        runOnce(() -> {
          intakeMotor.setPercent(percent);
        }),
        Commands.waitSeconds(.5),
        runOnce(() -> {
          intakeMotor.setPercent(0);
        }));
  }

  // TODO: Get proper angle for motor up position
  public Command rotateUp() {
    return Commands.runOnce(() -> {
        pivotMotor.setMotorPosition(360);
    });
  }

  // TODO: Get proper angle for motor down position
  public Command rotateDown() {
    return Commands.runOnce(() -> {
        pivotMotor.setMotorPosition(360);
    });
  }

  public Command intake() {
    return Commands.run(() -> {
      intakeMotor.setPercent(Constants.IntakeConstants.intakeSpeed);
    },
        this);
  }

  public Command stop() {
    return Commands.run(() -> {
      intakeMotor.setVoltage(0);
    },
        this);
  }
}