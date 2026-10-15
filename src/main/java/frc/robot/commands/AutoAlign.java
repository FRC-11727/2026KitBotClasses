// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.LimelightHelpers;

public class AutoAlign extends Command {
  private final CANDriveSubsystem drive;
  private final PIDController turnPID;

  // Proportional, Integral, and Derivative gains
  // Tune these values based on your physical robot's weight and wheel traction
  private static final double kP = 0.05;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  public AutoAlign(CANDriveSubsystem driveSystem) {
    this.drive = driveSystem;

    // Initialize PID controller aiming for a 0 degree offset (perfectly centered)
    this.turnPID = new PIDController(kP, kI, kD);
    this.turnPID.setSetpoint(0.0);

    // Set a tolerance so the command knows when it is "close enough" (e.g., within 1.5 degrees)
    this.turnPID.setTolerance(1.5);

    // Require the drive subsystem so no other drive commands run at the same time
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset the PID controller when the command starts
    turnPID.reset();
  }

  @Override
  public void execute() {
    // Check if a valid target is present[cite: 2]
    boolean hasTarget = LimelightHelpers.getTV("limelight");

    if (hasTarget) {
      // Get the horizontal offset from the crosshair to the target in degrees[cite: 2]
      double tx = LimelightHelpers.getTX("limelight");

      // Calculate the required rotation speed to get tx to 0
      double rotationSpeed = turnPID.calculate(tx);

      // Send the output to your drivetrain using your specific method[cite: 14]
      drive.driveArcade(0.0, rotationSpeed);
    } else {
      // If no target is in view, stop moving[cite: 14]
      drive.driveArcade(0.0, 0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the motors when the command finishes or the driver lets go of the button[cite: 14]
    drive.driveArcade(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    // Returns true if the robot is within the 1.5 degree tolerance
    return turnPID.atSetpoint();
  }
}