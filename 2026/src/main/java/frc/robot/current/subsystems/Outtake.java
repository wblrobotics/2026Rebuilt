package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSparkMax;

public class Outtake extends SubsystemBase{
    private MotorController intakeMotor;

    private final int rightMotorID = Constants.OuttakeConstants.motorID;

    public Outtake(String robotType) {
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
    }

    public void setVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }
    
    public Command launch() {    
        return Commands.sequence(
          runOnce(() -> {
            intakeMotor.setPercent(Constants.OuttakeConstants.outtake);
          }), 
          Commands.waitSeconds(.5), 
          runOnce(() -> {
            intakeMotor.setPercent(0);
          })
        );
    }



    public Command stop() {
        return Commands.run(() -> {
            intakeMotor.setVoltage(0);
        }, 
        this);
    }
    

    /** Returns a command that runs the intake continuously until the limit switches are pressed then stops. */
  // public Command continuousIntakeCommand() {
  //   return Commands.sequence(
  //     runOnce(() -> {
  //       intakeMotor.setPercent(Constants.IntakeConstants.intakeSpeed);
  //     }),
  //     Commands.waitUntil(() -> intakeMotor.hasCurrentReached() == true || aboveAmpThreshold() == true).withTimeout(10),
  //     runOnce(() -> {
  //       if (intakeMotor.hasCurrentReached()) {
  //       } else {
  //       }
  //     })
  //   );
  // }
}
