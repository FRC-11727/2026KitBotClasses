// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Spins the launcher up to its target RPM (velocity PID) while the feeder backs fuel away from
 * the wheel. Finishes once the launcher is up to speed, but leaves the rollers running so a
 * following {@link Launch} starts already spinning.
 */
public class SpinUp extends Command {
  CANFuelSubsystem fuelSubsystem;

  public SpinUp(CANFuelSubsystem fuelSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fuelSubsystem.spinUpLauncher();
    fuelSubsystem.setFeederRoller(
        SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
  }

  // Called once the command ends or is interrupted. The rollers are intentionally left
  // running so Launch can take over without a gap; the fuel subsystem's default command
  // stops them if nothing else does.
  @Override
  public void end(boolean interrupted) {
  }

  // Done once the launcher has reached its target speed.
  @Override
  public boolean isFinished() {
    return fuelSubsystem.atSpeed();
  }
}
