// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.TeleopDriveCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Autonomous", new ExampleAuto(driveSubsystem, fuelSubsystem));
    autoChooser.addOption("Velocity PID Test", velocityPidTest());
  }

  /**
   * Tuning/sim aid: steps the drivetrain through a few real speeds (m/s) using the velocity PID,
   * then stops. Select it as the auto, enable, and compare Drive/Left(Right)TargetMetersPerSec
   * against Drive/Left(Right)VelocityMetersPerSec in AdvantageScope.
   */
  private Command velocityPidTest() {
    return driveSubsystem
        .run(() -> driveSubsystem.driveClosedLoop(1.0, 1.0))
        .withTimeout(2.0)
        .andThen(driveSubsystem.run(() -> driveSubsystem.driveClosedLoop(2.0, 2.0)).withTimeout(2.0))
        .andThen(driveSubsystem.run(() -> driveSubsystem.driveClosedLoop(-1.0, -1.0)).withTimeout(2.0))
        .andThen(driveSubsystem.run(() -> driveSubsystem.driveClosedLoop(0.5, 1.5)).withTimeout(2.0))
        .finallyDo(driveSubsystem::stop)
        .withName("Velocity PID Test");
  }

  /** A DEFAULT command runs automatically whenever nothing else is using that subsystem. */
  private void configureDefaultCommands() {
    // Teleop arcade drive, always on unless an auto (or the turn demo) borrows the
    // drivetrain. The lambdas are read live every 20ms:
    //  - Left stick Y  = forward/back. Pushing a stick FORWARD reads NEGATIVE (a USB HID
    //    convention as old as flight sims), so we negate it. If your robot drives backwards,
    //    do NOT remove this minus sign; flip the motor inversions in Constants instead.
    //  - Right stick X = turning. WPILib treats counter-clockwise as positive, but drivers
    //    expect "push right = turn right (clockwise)", so this is negated too.
    //  - Right bumper  = slow mode while held.
    //  - Left bumper   = turbo mode while held.
    driveSubsystem.setDefaultCommand(
        new TeleopDriveCommand(
            driveSubsystem,
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX(),
            driverController.rightBumper(),
            driverController.leftBumper()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    operatorController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    operatorController.a().whileTrue(new Eject(fuelSubsystem));
    // While the right bumper on the driver controller is held, align the robot
    driverController.rightTrigger().whileTrue(new AutoAlign(driveSubsystem));
    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
