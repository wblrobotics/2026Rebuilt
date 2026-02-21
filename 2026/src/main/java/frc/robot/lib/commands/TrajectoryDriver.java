package frc.robot.lib.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.current.subsystems.swerveDrive.DriveConstants;
import frc.robot.lib.util.GeomUtil;

public class TrajectoryDriver extends Command {
    private final Drive drive;
    private final boolean slowMode;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ArrayList<Translation2d>> interiorSupplier;

    private boolean running = false;
    private final ProfiledPIDController driveController =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private Translation2d lastSetpointTranslation;

    private static double driveKp = DriveConstants.driveKp;
    private static double driveKd = DriveConstants.driveKd;
    private static double thetaKp = DriveConstants.turnKp;
    private static double thetaKd = DriveConstants.turnKd;
    private static double driveMaxVelocity;
    private static double driveMaxVelocitySlow;
    private static double driveMaxAcceleration;
    private static double thetaMaxVelocity;
    private static double thetaMaxVelocitySlow;
    private static double thetaMaxAcceleration;
    private static double driveTolerance = .04;
    private static double thetaTolerance = Units.degreesToRadians(1);
    private static double ffMinRadius;
    private static double ffMaxRadius;

    private Trajectory trajectory = new Trajectory();
    private TrajectoryConfig config = new TrajectoryConfig(DriveConstants.maxSpeedMetersPerSec, driveMaxAcceleration);
        

    public TrajectoryDriver(Drive drive, boolean slowMode, Pose2d finalPose, ArrayList<Translation2d> interiorWaypoints) {
        this(drive, slowMode, () -> finalPose, () -> interiorWaypoints);
    }

    public TrajectoryDriver(Drive drive, boolean slowMode, Supplier<Pose2d> poseSupplier, Supplier<ArrayList<Translation2d>> interiorSupplier) {
        this.drive = drive;
        this.slowMode = slowMode;
        this.poseSupplier = poseSupplier;
        this.interiorSupplier = interiorSupplier;
        addRequirements(drive);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        config.setKinematics(new SwerveDriveKinematics(DriveConstants.moduleTranslations));
        config.addConstraint(new MaxVelocityConstraint(slowMode ? driveMaxVelocitySlow : driveMaxVelocity));
        config.setStartVelocity(Math.hypot(drive.getFieldVelocity().vxMetersPerSecond, drive.getFieldVelocity().vyMetersPerSecond));
    }

    @Override
    public void initialize() {
        // Reset all controllers
        var initialPose = drive.getPose();
        driveController.reset(
            initialPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
            Math.min(
                0.0,
                -new Translation2d(drive.getFieldVelocity().vxMetersPerSecond, drive.getFieldVelocity().vyMetersPerSecond)
                    .rotateBy(poseSupplier.get().getTranslation().minus(drive.getPose().getTranslation()).getAngle().unaryMinus())
                    .getX()));
        thetaController.reset(initialPose.getRotation().getRadians(), drive.getFieldVelocity().omegaRadiansPerSecond);
        lastSetpointTranslation = drive.getPose().getTranslation();

        // Update constants
        driveController.setP(driveKp);
        driveController.setD(driveKd);
        driveController.setConstraints(new TrapezoidProfile.Constraints(slowMode ? driveMaxVelocitySlow : driveMaxVelocity, driveMaxAcceleration));
        driveController.setTolerance(driveTolerance);
        thetaController.setP(thetaKp);
        thetaController.setD(thetaKd);
        thetaController.setConstraints(new TrapezoidProfile.Constraints(slowMode ? thetaMaxVelocitySlow : thetaMaxVelocity, thetaMaxAcceleration));
        thetaController.setTolerance(thetaTolerance);

        var trajectory = TrajectoryGenerator.generateTrajectory(
        initialPose,
        interiorSupplier.get(),
        poseSupplier.get(),
        config);
    }

    @Override
    public void execute() {
        running = true;

        // Get current and target pose
        var currentPose = drive.getPose();
        var targetPose = poseSupplier.get();

        // Calculate drive speed
        double currentDistance = 
            currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
        double ffScalar = 
            MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
            lastSetpointTranslation.getDistance(targetPose.getTranslation()),
            driveController.getSetpoint().velocity);
        double driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScalar
            + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
        lastSetpointTranslation =
            new Pose2d(
                    targetPose.getTranslation(),
                    currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(
                    GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
                .getTranslation();
        
        // Calculate theta speed
        double thetaVelocity =
            thetaController.getSetpoint().velocity * ffScalar
                + thetaController.calculate(
                    currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs =
            Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        // Command speeds
        var driveVelocty =
            new Pose2d(
                    new Translation2d(),
                    currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocty.getX(), driveVelocty.getY(), thetaVelocity, currentPose.getRotation()));

        // Log data
        Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
        Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
        Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
        Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
        Logger.recordOutput("Odometry/DriveToPoseSetpoint", 
            new Pose2d(
                lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
        Logger.recordOutput("Odometry/DriveToPoseGoal", targetPose);

        Logger.recordOutput("TrajectoryTime", getTime());
    }

    @Override
    public void end(boolean interrupted) {
        running = false;
        drive.stop();
        Logger.recordOutput("Odometry/DriveToPoseSetpoint", new double[] {});
        Logger.recordOutput("Odometry/DriveToPoseGoal", new double[] {});
    }

    /** Checks if the robot is stopped at the final pose. */
    public boolean atGoal() {
        return running && driveController.atGoal() && thetaController.atGoal();
    }

    /** Checks if the robot pose is within the allowed drive and theta tolerances. */
    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
        return running
            && Math.abs(driveErrorAbs) < driveTolerance
            && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
    }

    /** Returns whether the command is actively running. */
    public boolean isRunning() {
        return running;
    }

    /** Returns how long it will take to run the trajectory. */
    public double getTime() {
        return trajectory.getTotalTimeSeconds();
    }
}
