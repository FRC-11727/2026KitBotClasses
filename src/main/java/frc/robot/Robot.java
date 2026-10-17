package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The robot's heartbeat. WPILib calls the methods below at the right moments: when the
 * robot boots, when a match phase starts, and every 20ms in between. Notice there's almost
 * no robot-specific code here: WHAT the robot is lives in RobotContainer; this file only
 * decides WHEN things run. You will rarely need to edit it.
 *
 * <p>We extend {@code LoggedRobot} (AdvantageKit's version of the standard TimedRobot) and
 * start the logger before anything else, so every value recorded with {@code
 * Logger.recordOutput(...)} or {@code @AutoLogOutput} is (1) published live to the dashboard
 * and (2) saved to a .wpilog file you can replay in AdvantageScope after a match. See
 * docs/05-logging-and-dashboards.md for the tour.
 */
public class Robot extends LoggedRobot {

  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  /** Runs once when the robot code boots. */
  public Robot() {
    // --- Log metadata: stamp every log file with exactly which code produced it ------------
    // BuildConstants is GENERATED at compile time by the gversion plugin (see build.gradle).
    // If your editor underlines it in red on a fresh clone, run a build once (WPILib icon ->
    // "Build Robot Code") and it will appear.
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    // A switch (not a ternary) because DIRTY is a compile-time constant: the compiler would
    // flag a ternary on it as dead code.
    switch (BuildConstants.DIRTY) {
      case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
      case 1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes");
      default -> Logger.recordMetadata("GitDirty", "Unknown");
    }

    // --- Where should the log data go? ------------------------------------------------------
    if (isReal()) {
      // On the real robot: write a .wpilog file AND stream live to the dashboard.
      // WPILOGWriter with no arguments saves to a USB stick plugged into the roboRIO
      // (strongly recommended; bring one to competitions), falling back to the roboRIO's
      // internal storage if none is present.
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      // In simulation: stream live (open AdvantageScope and connect to the simulator) and
      // also save a log file into the project's logs/ folder (gitignored).
      Logger.addDataReceiver(new WPILOGWriter("logs"));
      Logger.addDataReceiver(new NT4Publisher());
    }

    // NOTE for later: full AdvantageKit setups add a third mode here, REPLAY, which
    // re-runs a log file through the code deterministically. It requires structuring
    // subsystems with "IO layers", which is deliberately NOT done in this skeleton to keep
    // it approachable. When you're curious, see docs/08-next-steps.md.

    // Start logging. Everything before this line configures; nothing logs until it runs.
    Logger.start();

    // Build the actual robot (subsystems, buttons, autos).
    robotContainer = new RobotContainer();
  }

  /** Runs every 20ms, in EVERY mode. */
  @Override
  public void robotPeriodic() {
    // This single line is the engine of the whole command-based framework: it reads the
    // buttons, schedules and cancels commands, runs active commands, and calls every
    // subsystem's periodic(). Without it, no commands run and the robot does nothing.
    CommandScheduler.getInstance().run();
  }

  /** Runs once each time the robot becomes disabled (including after a match phase). */
  @Override
  public void disabledInit() {}

  /** Runs once when autonomous starts. */
  @Override
  public void autonomousInit() {
    // Ask the container which routine the drive team picked on the dashboard, and run it.
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      // Record which routine actually ran, which settles any post-match "wrong auto" debate.
      Logger.recordOutput("Robot/SelectedAuto", autonomousCommand.getName());
      // Commands are handed to the scheduler to run (calling schedule() directly on the
      // command is the pre-2026 style and is deprecated).
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** Runs once when teleop starts (after auto in a real match). */
  @Override
  public void teleopInit() {
    // Stop the auto routine when the driver takes over. Without this, an auto that hadn't
    // finished would keep fighting the driver for the drivetrain.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      autonomousCommand = null;
    }
    // Nothing to start here: TeleopDriveCommand is the drivetrain's DEFAULT command, so the
    // scheduler starts it automatically the moment nothing else claims the drivetrain.
  }

  /** Runs once when test mode starts (rarely used; fine to ignore for now). */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
