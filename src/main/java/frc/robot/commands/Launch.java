// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Launches fuel. The launcher is held at its target RPM by velocity PID, and the feeder only
 * pushes fuel into it once it is up to speed, so every shot leaves at the same speed. Until
 * then the feeder holds fuel back. If the launcher never reports being up to speed (e.g. a
 * dead encoder) it feeds anyway after SPIN_UP_TIMEOUT_SECONDS.
 */
public class Launch extends Command {
  CANFuelSubsystem fuelSubsystem;
  private final Timer timer = new Timer();

  public Launch(CANFuelSubsystem fuelSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
  }

  // Called when the command is initially scheduled. Start the launcher; hold fuel back.
  @Override
  public void initialize() {
    timer.restart();
    fuelSubsystem.spinUpLauncher();
    fuelSubsystem.setFeederRoller(
        SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean ready = fuelSubsystem.atSpeed() || timer.hasElapsed(SPIN_UP_TIMEOUT_SECONDS);
    if (ready) {
      fuelSubsystem.setFeederRoller(
          SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    } else {
      fuelSubsystem.setFeederRoller(
          SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
    }
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
