package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSparkMax;

public class Climber extends SubsystemBase{
    private MotorController leftClimbMotor;
    private MotorController rightClimbMotor;

    private final int leftClimbMotorID = Constants.ClimberConstants.leftClimbMotorID;
    private final int rightClimbMotorID = Constants.ClimberConstants.rightClimbMotorID;

    public Climber(){
        SparkMaxConfig leftClimbMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightClimbMotorConfig = new SparkMaxConfig();

        leftClimbMotorConfig.idleMode(IdleMode.kBrake);
        rightClimbMotorConfig.idleMode(IdleMode.kBrake);

        switch (Constants.robot) {
            case "Real":
                leftClimbMotor = new MotorController(new MotorIOSparkMax(leftClimbMotorID, leftClimbMotorConfig, 35), "Climber", "1");
                rightClimbMotor= new MotorController(new MotorIOSparkMax(rightClimbMotorID, rightClimbMotorConfig, 35), "Climber", "2");
                break;
            case "SIM":
                // Just Don't Use SIM    
                break;
            default:
                leftClimbMotor = new MotorController(new MotorIOSparkMax(leftClimbMotorID, leftClimbMotorConfig, 35), "Climber", "1");
                rightClimbMotor = new MotorController(new MotorIOSparkMax(rightClimbMotorID, rightClimbMotorConfig, 35), "Climber", "2");
                break;
        }
    }

    public void periodic(){
        leftClimbMotor.updateInputs();
        rightClimbMotor.updateInputs();
    }

}
