package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSparkMax;
import frc.robot.lib.motors.positionController.PositionController;
import frc.robot.lib.motors.positionController.PositionIOSparkMax;

public class Climber extends SubsystemBase{
    private MotorController leftClimbMotor;
    private MotorController rightClimbMotor;

    private PositionController leftPivotMotor;
    private PositionController rightPivotMotor;

    private final int leftClimbMotorID = Constants.ClimberConstants.leftClimbMotorID;
    private final int rightClimbMotorID = Constants.ClimberConstants.rightClimbMotorID;

    private final int leftPivotMotorID = Constants.ClimberConstants.leftPivotMotorID;
    private final int rightPivotMotorID = Constants.ClimberConstants.leftPivotMotorID;

    public Climber(){
        SparkMaxConfig leftClimbMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightClimbMotorConfig = new SparkMaxConfig();

        SparkMaxConfig leftPivotMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightPivotMotorConfig = new SparkMaxConfig();

        leftClimbMotorConfig.idleMode(IdleMode.kBrake);
        rightClimbMotorConfig.idleMode(IdleMode.kBrake);

        leftPivotMotorConfig.idleMode(IdleMode.kBrake);
        rightPivotMotorConfig.idleMode(IdleMode.kBrake);

        switch (Constants.robot) {
            case "Real":
                leftClimbMotor = new MotorController(new MotorIOSparkMax(leftClimbMotorID, leftClimbMotorConfig, 35), "Climber", "1");
                rightClimbMotor = new MotorController(new MotorIOSparkMax(rightClimbMotorID, rightClimbMotorConfig, 35), "Climber", "2");

                leftPivotMotor = new PositionController(new PositionIOSparkMax(leftPivotMotorID, leftPivotMotorConfig, 35), "Climber");
                rightPivotMotor = new PositionController(new PositionIOSparkMax(rightPivotMotorID, rightPivotMotorConfig, 35), "Climber");
                break;
            case "SIM":
                // Just Don't Use SIM    
                break;
            default:
                leftClimbMotor = new MotorController(new MotorIOSparkMax(leftClimbMotorID, leftClimbMotorConfig, 35), "Climber", "1");
                rightClimbMotor = new MotorController(new MotorIOSparkMax(rightClimbMotorID, rightClimbMotorConfig, 35), "Climber", "2");
                
                leftPivotMotor = new PositionController(new PositionIOSparkMax(leftPivotMotorID, leftPivotMotorConfig, 35), "Climber");
                rightPivotMotor = new PositionController(new PositionIOSparkMax(rightPivotMotorID, rightPivotMotorConfig, 35), "Climber");
                break;
        }
    }

    public void periodic(){
        leftClimbMotor.updateInputs();
        rightClimbMotor.updateInputs();
    }

}
