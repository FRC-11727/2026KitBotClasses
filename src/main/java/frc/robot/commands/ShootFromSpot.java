// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.ShotConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShotConstants.ShotSpot;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Shoots from one of the preset spots in {@link ShotSpot}, with no camera.
 *
 * <p>
 * The driver drives to the spot and points the robot at the hub. This command
 * spins the
 * launcher up to that spot's voltage, waits until it has had time to reach
 * speed AND the
 * robot has stopped moving, then feeds fuel until the button is released.
 *
 * <p>
 * It reads the drivetrain's speed but does NOT require the drivetrain, so the
 * driver can
 * keep making small aiming corrections while it runs. Feeding just waits until
 * they stop.
 */
public class ShootFromSpot extends Command {
  private final CANFuelSubsystem fuelSubsystem;
  private final CANDriveSubsystem driveSubsystem;
  private final ShotSpot spot;

  private final Timer spinUpTimer = new Timer();
  private Debouncer stillDebouncer;
  private double launcherVoltage;
  private double spinUpFeederVoltage;
  private double launchingFeederVoltage;
  private boolean feeding;

  public ShootFromSpot(CANFuelSubsystem fuelSystem, CANDriveSubsystem driveSystem, ShotSpot spot) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
    this.driveSubsystem = driveSystem;
    this.spot = spot;
    setName("Shoot from " + spot.label);
  }

  // Called when the command is initially scheduled. Read this spot's values from
  // the
  // dashboard and start the spin-up clock
  @Override
  public void initialize() {
    launcherVoltage = SmartDashboard.getNumber(spot.dashboardKey, spot.launcherVoltage);
    spinUpFeederVoltage = SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    launchingFeederVoltage = SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    // A fresh debouncer each time, so "still" always means still since THIS button
    // press
    stillDebouncer = new Debouncer(SETTLE_SECONDS, DebounceType.kRising);
    feeding = false;
    spinUpTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled. Hold the
  // feeder
  // back until we're ready, then feed
  @Override
  public void execute() {
    boolean spunUp = spinUpTimer.hasElapsed(SPIN_UP_SECONDS);
    boolean still = stillDebouncer.calculate(isRobotStill());

    // Once feeding starts it keeps going even if the robot gets bumped. Flipping
    // the feeder
    // back and forth mid-shot is a good way to jam fuel.
    if (spunUp && still) {
      feeding = true;
    }

    fuelSubsystem.setIntakeLauncherRoller(launcherVoltage);
    fuelSubsystem.setFeederRoller(feeding ? launchingFeederVoltage : spinUpFeederVoltage);

    Logger.recordOutput("Shot/Spot", spot.label);
    Logger.recordOutput("Shot/LauncherVoltage", launcherVoltage);
    Logger.recordOutput("Shot/SpunUp", spunUp);
    Logger.recordOutput("Shot/RobotStill", still);
    Logger.recordOutput("Shot/Feeding", feeding);
  }

  /** True when the robot is barely moving forward/back and barely turning. */
  private boolean isRobotStill() {
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    return Math.abs(speeds.vxMetersPerSecond) < MAX_SHOOTING_SPEED_METERS_PER_SEC
        && Math.abs(Units.radiansToDegrees(speeds.omegaRadiansPerSecond)) < MAX_SHOOTING_TURN_RATE_DEG_PER_SEC;
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
    Logger.recordOutput("Shot/Spot", "None");
    Logger.recordOutput("Shot/SpunUp", false);
    Logger.recordOutput("Shot/RobotStill", false);
    Logger.recordOutput("Shot/Feeding", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
