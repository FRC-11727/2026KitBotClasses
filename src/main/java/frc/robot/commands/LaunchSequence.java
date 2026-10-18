// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANFuelSubsystem;

/**
 * Spin the launcher up, then launch. The spin-up ends as soon as the launcher is at speed
 * (not after a fixed delay), with a timeout so a launcher that never gets there cannot hang
 * the sequence.
 */
public class LaunchSequence extends SequentialCommandGroup {
  /** Creates a new LaunchSequence. */
  public LaunchSequence(CANFuelSubsystem fuelSubsystem) {
    addCommands(
        new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_TIMEOUT_SECONDS),
        new Launch(fuelSubsystem));
  }
}
