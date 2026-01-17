package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSparkMax;



public class Intake extends SubsystemBase {
  private MotorController intakeMotor;

  private final int rightMotorID = Constants.IntakeConstants.motorID;

  public Intake(String robotType) {
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted(true);
    rightConfig.smartCurrentLimit(30);

    switch (robotType) {
      case "Real":
        intakeMotor = new MotorController(new MotorIOSparkMax(rightMotorID, rightConfig, 35), "Intake", "1");

        break;
      case "SIM":
        // Just don't use sim.

        break;
      default:
        intakeMotor = new MotorController(new MotorIOSparkMax(rightMotorID, rightConfig, 30), "Intake", "1");

        break;
    }
  }

  public void periodic() {
    intakeMotor.updateInputs();

    /* 
    // simple get of most recent value; if no value has been published,
    // returns the default value passed to the subscribe() function
    double val = dblSub.get();
    // get the most recent value; if no value has been published, returns
    // the passed-in default value
    double val = dblSub.get(-1.0);
    // subscribers also implement the appropriate Supplier interface, e.g. DoubleSupplier
    double val = dblSub.getAsDouble();
    // get the most recent value, along with its timestamp
    TimestampedDouble tsVal = dblSub.getAtomic();
    // read all value changes since the last call to readQueue/readQueueValues
    // readQueue() returns timestamps; readQueueValues() does not.
    TimestampedDouble[] tsUpdates = dblSub.readQueue();
    double[] valUpdates = dblSub.readQueueValues();
    */
  }


  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
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

  // public Boolean ballSpotted(){
  //       if(){
  //         return true;         
  //       } else {

  //     return false;
  //       }
  //   }
}
