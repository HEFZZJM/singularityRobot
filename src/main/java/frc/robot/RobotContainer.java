// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorArmSystemTestCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LinearMechanismTestCommand;
import frc.robot.commands.MechanismSystemArmTestCommand;
import frc.robot.commands.MotorTestCommand;
import frc.robot.subsystems.ElevatorArmSystemTestSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LinearMechanismTestSubsystem;
import frc.robot.subsystems.MechanismSystemArmTestSubsystem;
import frc.robot.subsystems.MotorTestSubsystem;
import frc.robot.subsystems.RotatingMechanismTestSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final MotorTestSubsystem m_motorTestSubsystem = new MotorTestSubsystem();
  private final RotatingMechanismTestSubsystem m_rotatingMechanismTestSubsystem =
      new RotatingMechanismTestSubsystem();
  private final LinearMechanismTestSubsystem m_linearMechanismTestSubsystem =
      new LinearMechanismTestSubsystem();
  private final MechanismSystemArmTestSubsystem m_msArmTestSubsystem =
      new MechanismSystemArmTestSubsystem();
  private final ElevatorArmSystemTestSubsystem m_elevArmSystemTestSubsystem =
      new ElevatorArmSystemTestSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // Schedule motor test command when A button is pressed
    m_driverController.a().whileTrue(new MotorTestCommand(m_motorTestSubsystem));
    // B: Run elevator+arm system-level FF test
    m_driverController.b()
        .whileTrue(new ElevatorArmSystemTestCommand(m_elevArmSystemTestSubsystem));
    m_driverController.x()
        .whileTrue(new LinearMechanismTestCommand(m_linearMechanismTestSubsystem));
    m_driverController.y().whileTrue(new MechanismSystemArmTestCommand(m_msArmTestSubsystem));
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
