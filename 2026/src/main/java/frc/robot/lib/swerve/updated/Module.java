package frc.robot.lib.swerve.updated;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.current.Constants;

import org.littletonrobotics.junction.Logger;

public class Module {
    private int index;
    private ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private static double wheelRadius = Units.inchesToMeters(1.95866);
    private double driveKp;
    private double driveKd;
    private double driveKs;
    private double driveKv;
    private double turnKp;
    private double turnKd;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.116970, 0.133240);
    private final PIDController driveFeedback =
        new PIDController(0.9, 0.0, 0.0, 0.02);
    private final PIDController turnFeedback =
        new PIDController(23.0, 0.0, 0.0, 0.02);

    public Module(ModuleIO io, int moduleNumber, PIDConfig drivePID, PIDConfig turnPID) {
        System.out.println("[Init] Creating module " + Integer.toString(moduleNumber));
        this.io = io;
        this.index = moduleNumber;

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);

        // These values have not yet been tuned. They are pulled from 6328's 2023 competition code. Expect changes to be required.
        switch (Constants.robot) {
            case "SIM":
                this.driveKp = 0.9;
                this.driveKd = 0.0;
                this.driveKs = 0.116970;
                this.driveKv = 0.133240;
                this.turnKp = 23.0;
                this.turnKd = 0.0;
                break;
            case "RealOld":
                this.driveKp = 0.1;
                this.driveKd = 0.0;
                this.driveKs = 0.18868;
                this.driveKv = 0.12825;
                this.turnKp = 4.0;
                this.turnKd = 0.0;
                break;
            case "Real":
                this.driveKp = drivePID.getKp();
                this.driveKd = drivePID.getKd();
                this.driveKs = drivePID.getKs();
                this.driveKv = drivePID.getKv();
                this.turnKp = turnPID.getKp(); // Old 4.0
                this.turnKd = turnPID.getKd();
        }

        driveFeedback.setPID(driveKp, 0.0, driveKd);
        turnFeedback.setPID(turnKp, 0.0, turnKd);
        driveFeedforward = new SimpleMotorFeedforward(driveKs, driveKv);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    }

    /**
     * Runs the module using setpoints provided by a SwerveModuleState while optimizing the angle.
     * 
     * @param state Setpoint in the form of a SwerveModuleState.
     * 
     * @return The optimized SwerveModuleState.
     */
    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        // Optimize state based on current angle
        // This avoids "taking the long road" to a module angle. (270 degrees to 0 degrees should only be a 90 degree route, not 270.)
        state.optimize(getAngle());
        //var optimizedState = SwerveModuleState.optimize(state, getAngle());

        // Run turn controller
        io.setTurnVoltage(
            turnFeedback.calculate(getAngle().getRadians(), state.angle.getRadians()));

        // Update velocity based on turn error
        state.speedMetersPerSecond *= Math.cos(turnFeedback.getError());

        // Run drive controller
        double velocityRadPerSec = state.speedMetersPerSecond / wheelRadius;
        io.setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

        return state;
    }

    /**
     * Sends a voltage value to the drive motor of a module while maintaining closed loop control with an angle setpoint of 0.0 on the turn motor.
     * 
     * @param volts Voltage to be sent to the drive motor.
     */
    public void runCharacterization(double volts) {
        // Closed loop control of module rotation
        io.setTurnVoltage(
            turnFeedback.calculate(getAngle().getRadians(), 0.0));

        // Open loop control of module drive
        io.setDriveVoltage(volts);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad));
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * wheelRadius;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * wheelRadius;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the drive wheel radius. */
    public static double getWheelRadius() {
        return wheelRadius;
    }
}
