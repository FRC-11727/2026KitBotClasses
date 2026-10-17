package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The command that drives the robot in teleop. This is the DEFAULT command of the drivetrain:
 * the scheduler runs it automatically whenever no other command (like an auto routine) is
 * using the drivetrain.
 *
 * <p>This file is also your tour of the COMMAND LIFECYCLE. Every command has four parts, and
 * the scheduler calls them for you:
 *
 * <ol>
 *   <li>{@link #initialize()} - runs ONCE when the command starts
 *   <li>{@link #execute()} - runs every 20ms while the command is scheduled
 *   <li>{@link #isFinished()} - checked after each execute(); return true to end
 *   <li>{@link #end(boolean)} - runs ONCE when the command ends or is interrupted
 * </ol>
 *
 * <p>All the "driver feel" shaping (deadband, squaring, slew limiting, slow mode) lives here
 * in one place, in plain sight, instead of buried in the subsystem or scattered around the
 * bindings. Tune it in Constants.OperatorConstants.
 *
 * <p>NOTE: Simple commands are often written inline these days ({@code drive.run(() -> ...)},
 * see Autos.java for that style). We write this one as a full class because it's the robot's
 * most important command and the class form makes the lifecycle explicit for learning.
 */
public class TeleopDriveCommand extends Command {

  // Suppliers, not plain doubles: a command is constructed ONCE at startup, but we need the
  // LIVE joystick value every loop. A DoubleSupplier is "a way to ask for the value later".
  private final CANDriveSubsystem drive;
  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier slowModeSupplier;
  private final BooleanSupplier turboModeSupplier;

  // Smooths the forward input so full-stick slams ramp over ~1/3 second instead of
  // instantly, which keeps the robot from tipping and the wheels from spinning. Turning is
  // deliberately NOT smoothed; drivers expect steering to be instant.
  private final SlewRateLimiter forwardLimiter =
      new SlewRateLimiter(OperatorConstants.FORWARD_SLEW_RATE);

  /**
   * @param drive the drivetrain to control
   * @param forwardSupplier live forward/backward input, +1 = full forward
   * @param rotationSupplier live rotation input, +1 = spin counter-clockwise
   * @param slowModeSupplier true while the driver holds the slow-mode button
   * @param turboModeSupplier true while the driver holds the turbo-mode button
   */
  public TeleopDriveCommand(
      CANDriveSubsystem drive,
      DoubleSupplier forwardSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier slowModeSupplier,
      BooleanSupplier turboModeSupplier) {
    this.drive = drive;
    this.forwardSupplier = forwardSupplier;
    this.rotationSupplier = rotationSupplier;
    this.slowModeSupplier = slowModeSupplier;
    this.turboModeSupplier = turboModeSupplier;
    // Tell the scheduler this command USES the drivetrain. Two commands that require the
    // same subsystem can never run at once; the newer one interrupts the older. This is
    // how the framework guarantees two pieces of code never fight over the same motors.
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Start the ramp from zero so the robot doesn't jump if the stick is already pushed
    // the instant teleop begins.
    forwardLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    // 1) Read the sticks (through the suppliers RobotContainer gave us).
    double forward = forwardSupplier.getAsDouble();
    double rotation = rotationSupplier.getAsDouble();

    // 2) Deadband: treat tiny off-center readings as zero so the robot doesn't creep.
    //    applyDeadband also re-scales the remaining range so 100% stick is still 100%.
    forward = MathUtil.applyDeadband(forward, OperatorConstants.DEADBAND);
    rotation = MathUtil.applyDeadband(rotation, OperatorConstants.DEADBAND);

    // 3) Square the inputs (keeping the sign). Human hands are much more precise in the
    //    lower half of the stick; squaring gives fine control at low speed while full
    //    stick still means full speed. (0.5 stick -> 0.25 power.)
    forward = Math.copySign(Math.pow(Math.abs(forward), OperatorConstants.INPUT_EXPONENT), forward);
    rotation = Math.copySign(Math.pow(Math.abs(rotation), OperatorConstants.INPUT_EXPONENT), rotation);

    // 4) Slow mode: while held, scale everything down for lining up precisely.
    if (slowModeSupplier.getAsBoolean()) {
      forward *= OperatorConstants.SLOW_MODE_MULTIPLIER;
      rotation *= OperatorConstants.SLOW_MODE_MULTIPLIER;
    }

    if (!turboModeSupplier.getAsBoolean()) {
      forward *= OperatorConstants.NORMAL_SPEED_MULTIPLIER;
      rotation *= OperatorConstants.NORMAL_SPEED_MULTIPLIER;
    }

    // 5) Rate-limit the forward command (see field comment above).
    forward = forwardLimiter.calculate(forward);

    // 6) Finally, tell the drivetrain what to do.
    drive.driveArcade(forward, rotation);

    // Log what the driver is ASKING for. Comparing this against Drive/AppliedOutput* in a
    // log answers the classic question "was it the code, the driver, or the robot?"
    Logger.recordOutput("TeleopDrive/ForwardCommand", forward);
    Logger.recordOutput("TeleopDrive/RotationCommand", rotation);
    Logger.recordOutput("TeleopDrive/SlowMode", slowModeSupplier.getAsBoolean());
    Logger.recordOutput("TeleopDrive/RawForward", forwardSupplier.getAsDouble());
    //Logger.recordOutput("TeleopDrive/RawRotation", rotationSupplier.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    // A default command should never finish on its own; it drives whenever nothing more
    // important (like an autonomous routine) has claimed the drivetrain.
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Leave the motors stopped whether we ended normally or something interrupted us.
    drive.stop();
  }
}
