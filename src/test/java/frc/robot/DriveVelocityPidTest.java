package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Runs the drivetrain in the physics simulation (no robot needed) and checks that the
 * velocity PID in {@link CANDriveSubsystem#driveClosedLoop} actually reaches its target speed.
 * Run with: ./gradlew test
 */
class DriveVelocityPidTest {
  private static final double DT = 0.02; // one robot loop

  // One subsystem for the whole class: REVLib allows only one SparkMax per CAN ID, and a
  // closed one leaves its simulated device unusable. Each test starts by driving to a stop.
  private static CANDriveSubsystem drive;

  @BeforeAll
  static void initSim() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    drive = new CANDriveSubsystem();
  }

  @AfterAll
  static void closeSim() {
    drive.close();
  }

  @BeforeEach
  void settleAtZero() {
    run(0.0, 0.0, 3.0);
  }

  /** One robot loop: command the drive, then step the subsystem and its physics. */
  private static void step(double leftMps, double rightMps) {
    drive.driveClosedLoop(leftMps, rightMps);
    drive.periodic();
    drive.simulationPeriodic();
  }

  private static void run(double leftMps, double rightMps, double seconds) {
    for (int i = 0; i < (int) (seconds / DT); i++) {
      step(leftMps, rightMps);
    }
  }

  @Test
  void reachesForwardTargetSpeed() {
    run(1.5, 1.5, 3.0);
    assertEquals(1.5, drive.getLeftVelocityMps(), 0.15, "left should settle near target");
    assertEquals(1.5, drive.getRightVelocityMps(), 0.15, "right should settle near target");
  }

  @Test
  void reachesReverseTargetSpeed() {
    run(-1.0, -1.0, 3.0);
    assertEquals(-1.0, drive.getLeftVelocityMps(), 0.15);
    assertEquals(-1.0, drive.getRightVelocityMps(), 0.15);
  }

  @Test
  void differentSideSpeedsTurnTheRobot() {
    run(0.5, 1.5, 3.0);
    assertEquals(0.5, drive.getLeftVelocityMps(), 0.15);
    assertEquals(1.5, drive.getRightVelocityMps(), 0.15);
  }

  @Test
  void targetsAboveTopSpeedAreClamped() {
    run(100.0, 100.0, 4.0);
    double speed = drive.getLeftVelocityMps();
    assertTrue(speed <= DriveConstants.MAX_SPEED_MPS + 0.2, "must not exceed top speed: " + speed);
    assertTrue(speed >= DriveConstants.MAX_SPEED_MPS * 0.8, "should be near top speed: " + speed);
  }

  @Test
  void stopsWhenTargetIsZero() {
    run(1.5, 1.5, 2.0);
    run(0.0, 0.0, 3.0);
    assertEquals(0.0, drive.getLeftVelocityMps(), 0.1);
    assertEquals(0.0, drive.getRightVelocityMps(), 0.1);
  }
}
