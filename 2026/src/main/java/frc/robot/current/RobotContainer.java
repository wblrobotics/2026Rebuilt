// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.current;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.current.Constants.OperatorConstants;
import frc.robot.current.subsystems.ExamplePivot;
import frc.robot.current.subsystems.Intake;
import frc.robot.current.subsystems.LedOperation;
import frc.robot.current.subsystems.Outtake;
import frc.robot.lib.commands.DriveWithController;
import frc.robot.lib.swerve.updated.GyroIONavX2;
import frc.robot.lib.swerve.updated.ModuleConfig;
import frc.robot.lib.swerve.updated.ModuleType;
import frc.robot.lib.swerve.updated.PIDConfig;
import frc.robot.lib.swerve.updated.SwerveDrive;
import frc.robot.lib.vision.VisionIOPhotonVision;
import frc.robot.lib.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private ExamplePivot exPivot;
  private SwerveDrive swerveDrive;
  private LedOperation leds;
  private Intake intake;
  private Vision vision;
  private Outtake outtake;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController controlXbox = new CommandXboxController(OperatorConstants.kOtherControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    leds = new LedOperation();
    exPivot = new ExamplePivot(Constants.robot);
    intake = new Intake(Constants.robot);
    outtake = new Outtake(Constants.robot);

    swerveDrive = new SwerveDrive(
        24.5,
        24.5,
        new PIDConfig(0.1, 0.0, 0.0, 0.18868, 0.12825),
        new PIDConfig(4.0, 0.0, 0.0, 0.0, 0.0),
        new GyroIONavX2() {
        }, "Spark Max",
        ModuleType.SDSMK4iL3,
        new ModuleConfig(1, 2, 9, 0.0),
        new ModuleConfig(3, 4, 10, 0.0),
        new ModuleConfig(5, 6, 11, 0.0),
        new ModuleConfig(7, 8, 12, 0));

        vision = new Vision(
          // Left Module
          new VisionIOPhotonVision("Left Module", 
            new Pose3d(
              Units.inchesToMeters(7.375),
              Units.inchesToMeters(10.875),
              Units.inchesToMeters(12),
              new Rotation3d(0, Units.degreesToRadians(-11), Units.degreesToRadians(-11)))
          ),
          // Right Module
          new VisionIOPhotonVision("Right Module", 
            new Pose3d(
              Units.inchesToMeters(7.375),
              Units.inchesToMeters(-10.875),
              Units.inchesToMeters(11.5),
              new Rotation3d(0, Units.degreesToRadians(11), Units.degreesToRadians(11)))
          ),
          // Backup Cam
          new VisionIOPhotonVision("Backup Cam", 
            new Pose3d(
              Units.inchesToMeters(-0.5),
              Units.inchesToMeters(-5),
              Units.inchesToMeters(37.5),
              new Rotation3d(5, 0, 175))
          ));
    

    vision.setDataInterfaces(swerveDrive::addVisionData, swerveDrive::getPose);
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {


        exPivot.setDefaultCommand(
          Commands.run(() -> {
            exPivot.adjustHeight(-1 * controlXbox.getLeftY());
          },
            exPivot));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    controlXbox.a().whileTrue(intake.intake()).onFalse(intake.stop());
    controlXbox.x().onTrue(outtake.launch());

    swerveDrive.setDefaultCommand(
        new DriveWithController(swerveDrive, 0.5, 0.25, () -> driveXbox.getLeftX(), () -> driveXbox.getLeftY(),
            () -> driveXbox.getRightX(), () -> driveXbox.getRightY(), () -> driveXbox.a().getAsBoolean()));
    driveXbox.x().onTrue(Commands.runOnce(swerveDrive::stopWithX, swerveDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
