package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.FuelConstants;
import frc.robot.commands.Launch;
import frc.robot.commands.SpinUp;
import frc.robot.subsystems.CANFuelSubsystem;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Runs the launcher in the physics simulation (no robot needed) and checks that the velocity
 * PID reaches its target RPM, and that the commands only feed fuel once it is up to speed.
 * Run with: ./gradlew test
 */
class ShooterVelocityPidTest {
  private static final double DT = 0.02; // one robot loop
  private static final double TARGET_RPM = FuelConstants.LAUNCHER_TARGET_RPM;
  private static final double TOLERANCE = FuelConstants.LAUNCHER_READY_TOLERANCE_RPM;

  // One subsystem for the whole class: REVLib allows only one SparkMax per CAN ID, and a
  // closed one leaves its simulated device unusable. Each test starts by stopping.
  private static CANFuelSubsystem fuel;

  @BeforeAll
  static void initSim() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    fuel = new CANFuelSubsystem();
  }

  @AfterAll
  static void closeSim() {
    fuel.close();
  }

  @BeforeEach
  void settleAtZero() {
    fuel.stop();
    for (int i = 0; i < 200; i++) {
      step();
    }
  }

  /** One robot loop: step the subsystem and its flywheel physics. */
  private static void step() {
    fuel.periodic();
    fuel.simulationPeriodic();
  }

  private static void run(double seconds) {
    for (int i = 0; i < (int) (seconds / DT); i++) {
      step();
    }
  }

  @Test
  void reachesTargetRpm() {
    fuel.setLauncherRpm(TARGET_RPM);
    run(3.0);
    assertEquals(TARGET_RPM, fuel.getLauncherRpm(), TOLERANCE, "should settle near target");
    assertTrue(fuel.atSpeed());
  }

  @Test
  void reachesLowerTargetToo() {
    fuel.setLauncherRpm(3000.0);
    run(3.0);
    assertEquals(3000.0, fuel.getLauncherRpm(), TOLERANCE);
  }

  @Test
  void notAtSpeedBeforeSpinUp() {
    assertFalse(fuel.atSpeed(), "stopped launcher must not count as at speed");
    fuel.setLauncherRpm(TARGET_RPM);
    step();
    assertFalse(fuel.atSpeed(), "one loop after commanding is nowhere near speed yet");
  }

  @Test
  void stopsWhenStopped() {
    fuel.setLauncherRpm(TARGET_RPM);
    run(3.0);
    fuel.stop();
    run(3.0);
    assertEquals(0.0, fuel.getLauncherRpm(), TOLERANCE);
    assertFalse(fuel.atSpeed());
  }

  @Test
  void voltageModeLeavesSpeedMode() {
    fuel.setLauncherRpm(TARGET_RPM);
    run(3.0);
    assertTrue(fuel.atSpeed());
    // Intaking/ejecting use voltage; the launcher must no longer report "spun up".
    fuel.setIntakeLauncherRoller(10.0);
    assertFalse(fuel.atSpeed());
    assertEquals(0.0, fuel.getLauncherTargetRpm());
  }

  @Test
  void launchHoldsFuelBackUntilAtSpeedThenFeeds() {
    Launch launch = new Launch(fuel);
    launch.initialize();
    launch.execute();
    assertFalse(fuel.atSpeed());
    assertEquals(
        FuelConstants.SPIN_UP_FEEDER_VOLTAGE,
        fuel.getFeederVoltage(),
        1e-9,
        "before the launcher is up to speed the feeder must hold fuel back");

    // Run until the launcher reaches speed (well inside the fallback timeout).
    int loops = 0;
    while (!fuel.atSpeed() && loops < 100) {
      step();
      launch.execute();
      loops++;
    }
    assertTrue(fuel.atSpeed(), "launcher should reach speed");
    assertTrue(loops * DT < FuelConstants.SPIN_UP_TIMEOUT_SECONDS, "reached speed on its own");

    step();
    launch.execute();
    assertEquals(
        FuelConstants.LAUNCHING_FEEDER_VOLTAGE,
        fuel.getFeederVoltage(),
        1e-9,
        "once at speed the feeder pushes fuel into the launcher");

    launch.end(false);
    assertEquals(0.0, fuel.getFeederVoltage(), 1e-9, "ending Launch stops the feeder");
  }

  @Test
  void spinUpFinishesWhenAtSpeed() {
    SpinUp spinUp = new SpinUp(fuel);
    spinUp.initialize();
    assertFalse(spinUp.isFinished(), "not done right after starting");
    int loops = 0;
    while (!spinUp.isFinished() && loops < 200) {
      step();
      loops++;
    }
    assertTrue(spinUp.isFinished(), "done once the launcher is up to speed");
    assertTrue(fuel.atSpeed());
  }
}
