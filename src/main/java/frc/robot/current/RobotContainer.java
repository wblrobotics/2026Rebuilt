// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.current;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.current.Constants.OperatorConstants;
import frc.robot.current.commands.Autos;
import frc.robot.current.commands.ExampleCommand;
import frc.robot.current.subsystems.ExampleSubsystem;
import frc.robot.lib.swerve.updated.GyroIO;
import frc.robot.lib.swerve.updated.GyroIOADXRS450;
import frc.robot.lib.swerve.updated.ModuleConfig;
import frc.robot.lib.swerve.updated.ModuleIOSim;
import frc.robot.lib.swerve.updated.ModuleIOSparkMax;
import frc.robot.lib.swerve.updated.ModuleType;
import frc.robot.lib.swerve.updated.PIDConfig;
import frc.robot.lib.swerve.updated.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private SwerveDrive swerveDrive;
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // TODO: Get better trackWidth measurments
    switch (Constants.robot) {
      case "SIM":
        swerveDrive = new SwerveDrive(
            21, 
            21, 
            new PIDConfig(0.9, 0.0, 0.0, 0.116970, 0.133240), 
            new PIDConfig(23, 0.0, 0.0, 0.0, 0.0), 
            new GyroIO() {},
            ModuleType.SDSMK4iL3, 
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
        break;
      case "Real":
        swerveDrive = new SwerveDrive(
            21, 
            21, 
            new PIDConfig(0.1, 0.0, 0.0, 0.18868, 0.12825), 
            new PIDConfig(4.0, 0.0, 0.0, 0.0, 0.0), 
            new GyroIOADXRS450() {},
            ModuleType.SDSMK4iL3, 
            new ModuleIOSparkMax(0, ModuleType.SDSMK4iL3, new ModuleConfig(1, 2, 9, 0.0)),
            new ModuleIOSparkMax(1, ModuleType.SDSMK4iL3, new ModuleConfig(3, 4, 10, 0.0)),
            new ModuleIOSparkMax(2, ModuleType.SDSMK4iL3, new ModuleConfig(5, 6, 11, 0.0)),
            new ModuleIOSparkMax(3, ModuleType.SDSMK4iL3, new ModuleConfig(7, 8, 12, 0.0)));
        break;
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
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
