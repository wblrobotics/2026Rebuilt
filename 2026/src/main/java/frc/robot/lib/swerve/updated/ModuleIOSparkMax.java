package frc.robot.lib.swerve.updated;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.CleanSparkMaxValue;
import frc.robot.lib.util.SparkMaxPeriodicFrameConfig;

public class ModuleIOSparkMax implements ModuleIO {
    private SparkMax driveSparkMax;
    private SparkMax turnSparkMax;
    private SparkMaxConfig driveConfig = new SparkMaxConfig();
    private SparkMaxConfig turnConfig = new SparkMaxConfig();

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnRelativeEncoder;

    // I've opted to also add our CANCoders in here for the time being. These can be
    // moved if we see a need, although I don't see us mounting anything else on
    // these modules right now
    private CANcoder turnAbsoluteEncoder;

    // SDS MK4i L3 Gearing
    private double driveAfterEncoderReduction;
    private double turnAfterEncoderReduction;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    /**
     * 
     * @param index        The ID the modules are coded as. Front left is 0, fr 1,
     *                     bl 2, br 3.
     * @param moduleType   This is again the model of module you are using
     * @param moduleConfig This is where you set constants for each module
     */
    public ModuleIOSparkMax(int index, ModuleType moduleType, ModuleConfig moduleConfig) {
        System.out.println("[Init] Creating ModuleIOSparkMax" + Integer.toString(index));

        driveAfterEncoderReduction = moduleType.driveReduction();
        turnAfterEncoderReduction = moduleType.turnReduction();

        if (index <= 3 && index >= 0) {
            driveSparkMax = new SparkMax(moduleConfig.getDriveID(), MotorType.kBrushless);
            turnSparkMax = new SparkMax(moduleConfig.getTurnID(), MotorType.kBrushless);
            turnAbsoluteEncoder = new CANcoder(moduleConfig.getEncoderID());
            absoluteEncoderOffset = new Rotation2d(moduleConfig.getEncoderOffset());
        } else {
            throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
        }

        // Insert burn logic here...

        driveSparkMax.setCANTimeout(500);
        turnSparkMax.setCANTimeout(500);

        driveEncoder = driveSparkMax.getEncoder();
        turnRelativeEncoder = turnSparkMax.getEncoder();

        // Spark Max Config
        turnConfig.inverted(isTurnMotorInverted);

        driveConfig.smartCurrentLimit(40);
        turnConfig.smartCurrentLimit(30);

        driveConfig.voltageCompensation(12);
        turnConfig.voltageCompensation(12.0);
        // We'll set these values in memory. We aren't burning for now. This can be
        // updated later...
        for (int i = 0; i < 2; i++) {
            SparkMaxPeriodicFrameConfig.configNotLeader(driveSparkMax, driveConfig);
            SparkMaxPeriodicFrameConfig.configNotLeader(turnSparkMax, turnConfig);
            driveConfig.signals.primaryEncoderPositionPeriodMs(10);

            // TODO: Look at encoder stuff - why can't they be together?
            driveEncoder.setPosition(0.0);
            driveConfig.absoluteEncoder.averageDepth(2);
            // driveConfig.alternateEncoder.measurementPeriod(10);

            turnRelativeEncoder.setPosition(0.0);
            // turnConfig.alternateEncoder.measurementPeriod(10);
            turnConfig.absoluteEncoder.averageDepth(2);
            // turnConfig.absoluteEncoder.inverted(true);

        }

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        driveSparkMax.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnSparkMax.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = CleanSparkMaxValue.cleanSparkMaxValue(
                inputs.drivePositionRad,
                Units.rotationsToRadians(driveEncoder.getPosition()) / driveAfterEncoderReduction);
        inputs.driveVelocityRadPerSec = CleanSparkMaxValue.cleanSparkMaxValue(
                inputs.driveVelocityRadPerSec,
                Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / driveAfterEncoderReduction);
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = new double[] { driveSparkMax.getOutputCurrent() };
        // inputs.driveTempCelcius = new double[] {driveSparkMax.getMotorTemperature()};

        inputs.turnAbsolutePositionRad = MathUtil.angleModulus(
                new Rotation2d(
                        turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()
                                * 2.0
                                * Math.PI)
                        .minus(absoluteEncoderOffset)
                        .getRadians());
        inputs.turnPositionRad = CleanSparkMaxValue.cleanSparkMaxValue(
                inputs.turnPositionRad,
                Units.rotationsToRadians(turnRelativeEncoder.getPosition())
                        / turnAfterEncoderReduction);
        inputs.turnVelocityRadPerSec = CleanSparkMaxValue.cleanSparkMaxValue(
                inputs.turnVelocityRadPerSec,
                Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
                        / turnAfterEncoderReduction);
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[] { turnSparkMax.getOutputCurrent() };
        // inputs.turnTempCelcius = new double[] {turnSparkMax.getMotorTemperature()};
    }

    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
    }

    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    public void setDriveBrakeMode(boolean enable) {
        driveConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void setTurnBrakeMode(boolean enable) {
        turnConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
