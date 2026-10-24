package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.AutoConstants.*;

public class ShootAllCommand extends SequentialCommandGroup {
  
  public ShootAllCommand(CANFuelSubsystem fuelSubsystem) {
    
    addCommands(
      // Step 1: Spin up the launcher wheel to full speed
      new RunCommand(() -> fuelSubsystem.setIntakeLauncherRoller(LAUNCHING_LAUNCHER_VOLTAGE), fuelSubsystem)
          .withTimeout(1.0), // Give it 1 second to reach full speed

      // Step 2: Keep the launcher spinning, but also turn on the feeder to push the balls in
      new RunCommand(() -> {
            fuelSubsystem.setIntakeLauncherRoller(LAUNCHING_LAUNCHER_VOLTAGE);
            fuelSubsystem.setFeederRoller(LAUNCHING_FEEDER_VOLTAGE);
          }, fuelSubsystem)
          .withTimeout(20.0), // Run both for 20 seconds to ensure all balls are shot

      // Step 3: Stop all fuel motors
      new RunCommand(() -> fuelSubsystem.stop(), fuelSubsystem)
          .withTimeout(0.1)
    );
  }
}