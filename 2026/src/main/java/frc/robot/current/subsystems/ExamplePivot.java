package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.Constants.PivotConstants;
import frc.robot.lib.motors.positionController.PositionController;
import frc.robot.lib.motors.positionController.PositionIOSim;
import frc.robot.lib.motors.positionController.PositionIOSparkMax;

public class ExamplePivot extends SubsystemBase{
    private PositionController motor;

    private final int motorID = Constants.PivotConstants.algaePivotID;

    private ProfiledPIDController motorPIDController;
    private TrapezoidProfile.Constraints motorConstraints;
    private final ArmFeedforward motorFeedForward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV, PivotConstants.kA);

    private Boolean inDownPosition = false;

    private double requestedPivotGoal;
    // TODO: TEST THESE VALUES
    private double minAngle = PivotConstants.minAngle;
    private double maxAngle = PivotConstants.maxAngle;

    public ExamplePivot(String robotType) {

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(40);

        motorConfig.idleMode(IdleMode.kBrake);
        motorConstraints = new TrapezoidProfile.Constraints(60, 30);
        motorPIDController = new ProfiledPIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, motorConstraints);

        //TODO: Get real PID, constraints, and motor feedforward stuff;
        switch (robotType) {
            case "Real":
                motor = new PositionController(new PositionIOSparkMax(motorID, motorConfig, PivotConstants.pivotOffset), "Algae");

                break;
            case "Sim":
                motor = new PositionController(new PositionIOSim(), "Algae");

                break;
            default:
                motor = new PositionController(new PositionIOSparkMax(motorID, motorConfig, PivotConstants.pivotOffset), "Algae");

                break;
        }

        requestedPivotGoal = PivotConstants.pivotStowAway - 5;
        motor.setEnabled(false);
    }

    public void periodic() {
        motor.updateInputs();
        Logger.recordOutput("AlgeaPivot/GoalVelocity", motorPIDController.getGoal().velocity);
        Logger.recordOutput("AlgaePivot/Goal", motorPIDController.getGoal().position);
        Logger.recordOutput("AlgaePivot/PIDOutput", motorPIDController.calculate(motor.getAngle())
        + motorFeedForward.calculate(motor.getAngle(), motorPIDController.getGoal().velocity));

        double outputGoal = MathUtil.clamp(requestedPivotGoal, minAngle, maxAngle);
        motorPIDController.setGoal(outputGoal);
        
        motor.setVoltage(
            motorPIDController.calculate(motor.getAngle())
                + motorFeedForward.calculate(motor.getAngle(), motorPIDController.getGoal().velocity)
            );
        
        Logger.recordOutput("AlgaePivot/RequestedGoal", requestedPivotGoal);
    }

    /**
     * Method to set the pivot goal to the current position
     */
    public void initialization() {
        requestedPivotGoal = motor.getAngle();
    }

    /**
     * @return The requested goal to set the pivot at
     */
    public double getRequestedGoal() {
        return requestedPivotGoal;
    }

    public void adjustHeight(double percent) {
        double increaseFactor = percent / 5 ;
        double currentHeight = requestedPivotGoal;

        requestedPivotGoal = currentHeight + increaseFactor;
    }

    public void setVoltage(double percent) {
        motor.setVoltage(percent * 12);
    }

    public double getAlgaeAngle() {
        return motor.getAngle();
    }

    public Boolean isDown() {
        return inDownPosition;
    }
}
