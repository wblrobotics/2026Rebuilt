// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.current;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.current.Constants.OperatorConstants;
import frc.robot.current.commands.Autos;
import frc.robot.current.commands.ExampleCommand;
import frc.robot.current.subsystems.ExampleSubsystem;
import frc.robot.current.subsystems.LedOperation;
import frc.robot.lib.commands.DriveWithController;
import frc.robot.lib.swerve.updated.GyroIO;
import frc.robot.lib.swerve.updated.GyroIOADXRS450;
import frc.robot.lib.swerve.updated.ModuleConfig;
import frc.robot.lib.swerve.updated.ModuleIOSim;
import frc.robot.lib.swerve.updated.ModuleIOSparkMax;
import frc.robot.lib.swerve.updated.ModuleType;
import frc.robot.lib.swerve.updated.PIDConfig;
import frc.robot.lib.swerve.updated.SwerveDrive;

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
  private SwerveDrive swerveDrive;
  private LedOperation leds;
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    leds = new LedOperation();
    swerveDrive = new SwerveDrive(
        20.75,
        20.75,
        new PIDConfig(0.1, 0.0, 0.0, 0.18868, 0.12825),
        new PIDConfig(4.0, 0.0, 0.0, 0.0, 0.0),
        new GyroIOADXRS450() {
        },
        ModuleType.SDSMK4iL3,
        new ModuleConfig(1, 2, 9, 0.0),
        new ModuleConfig(3, 4, 10, 0.0),
        new ModuleConfig(5, 6, 11, 0.0),
        new ModuleConfig(7, 8, 12, 2));

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    driveXbox.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

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
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
