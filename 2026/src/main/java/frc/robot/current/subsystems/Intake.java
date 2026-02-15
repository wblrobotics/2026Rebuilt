package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkFlexConfig;
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
import frc.robot.lib.motors.positionController.PositionController;
import frc.robot.lib.motors.positionController.PositionIOSparkFlex;

public class Intake extends SubsystemBase {
  private MotorController intakeMotor;
  private PositionController pivotMotor;

  private final int rightMotorID = Constants.IntakeConstants.motorID;
  private final int pivotMotorID = Constants.IntakeConstants.pivotMotorID;
  
  public Intake(String robotType, Drive drive) {

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted(true);
    rightConfig.smartCurrentLimit(30);

    SparkFlexConfig pivotConfig = new SparkFlexConfig();
    pivotConfig.inverted(false);
    pivotConfig.smartCurrentLimit(30);

    pivotConfig.closedLoop
            .p(IntakeConstants.kP)
            .i(IntakeConstants.kI)
            .d(IntakeConstants.kD)
        .feedForward // Set Feedforward gains for the velocity controller
            .kS(IntakeConstants.kS) // Static gain (volts)
            .kV(IntakeConstants.kV) // Velocity gain (volts per RPM)
            .kA(IntakeConstants.kA); // Acceleration gain (volts per RPM/s)

    switch (robotType) {
      case "Real":
        intakeMotor = new MotorController(new MotorIOSparkFlex(rightMotorID, rightConfig, 35), "Intake", "1");
        pivotMotor = new PositionController(new PositionIOSparkFlex(pivotMotorID, pivotConfig, 0.0),"Intake");
        break;
      case "SIM":
        // Just don't use sim.

        break;
      default:
        intakeMotor = new MotorController(new MotorIOSparkMax(rightMotorID, rightConfig, 30), "Intake", "1");
        pivotMotor = new PositionController(new PositionIOSparkFlex(pivotMotorID, pivotConfig,0.0), "Intake");
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

  public Command rotateUp() {
    return Commands.runOnce(() -> {
        pivotMotor.setMotorPosition(Constants.IntakeConstants.upAngle);
    });
  }

  public Command rotateDown() {
    return Commands.runOnce(() -> {
        pivotMotor.setMotorPosition(Constants.IntakeConstants.downAngle);
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