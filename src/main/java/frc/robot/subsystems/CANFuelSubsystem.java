// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;

  // The launcher's built-in PID. We give it an RPM target and it holds that speed.
  private final SparkClosedLoopController launcherController;

  // Live copies of the gains so they can be tuned from SmartDashboard without redeploying.
  private double launcherKp = LAUNCHER_KP;
  private double launcherKv = LAUNCHER_KV;

  // RPM the launcher is currently told to hold, or 0 when it is under voltage/stopped.
  private double launcherTargetRpm = 0.0;

  // The voltage last sent to the feeder (the feeder is not closed-loop).
  private double feederVoltage = 0.0;

  // Simulation: a flywheel physics model backfills the launcher's encoder.
  private final SparkMaxSim launcherSim;
  private final FlywheelSim launcherPhysics =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), LAUNCHER_SIM_MOI_KG_M2, 1.0),
          DCMotor.getNEO(1));

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushless motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    launcherController = intakeLauncherRoller.getClosedLoopController();

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, add velocity PID (RPM, built-in encoder), and apply the config
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(launcherKp)
        .d(LAUNCHER_KD)
        .outputRange(-1.0, 1.0);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Without this the simulated SPARK MAX ignores all commands. It makes it follow the
    // Driver Station's enabled/disabled state, like real hardware.
    launcherSim = new SparkMaxSim(intakeLauncherRoller, DCMotor.getNEO(1));
    launcherSim.useDriverStationEnable();

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

    // Launcher PID tuning: the target RPM and gains can be changed live.
    SmartDashboard.putNumber("Launcher/TargetRPM", LAUNCHER_TARGET_RPM);
    SmartDashboard.putNumber("Launcher/kP", launcherKp);
    SmartDashboard.putNumber("Launcher/kV", launcherKv);
  }

  // A method to set the voltage of the intake roller (used for intaking and ejecting).
  // This leaves velocity mode, so the launcher no longer counts as "spun up".
  public void setIntakeLauncherRoller(double voltage) {
    launcherTargetRpm = 0.0;
    intakeLauncherRoller.setVoltage(voltage);
  }

  // A method to set the voltage of the feeder roller
  public void setFeederRoller(double voltage) {
    feederVoltage = voltage;
    feederRoller.setVoltage(voltage);
  }

  /**
   * Hold the launcher at a speed using velocity PID on the SPARK MAX.
   *
   * <p>Output = kV * target (feedforward, the bulk of the power) + kP * (target - actual).
   *
   * @param rpm target launcher speed in motor RPM (positive = launching direction)
   */
  public void setLauncherRpm(double rpm) {
    launcherTargetRpm = rpm;
    launcherController.setSetpoint(
        rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, launcherKv * rpm, ArbFFUnits.kVoltage);
  }

  /** Spin the launcher up to the target RPM currently set on the dashboard (Constants default). */
  public void spinUpLauncher() {
    setLauncherRpm(SmartDashboard.getNumber("Launcher/TargetRPM", LAUNCHER_TARGET_RPM));
  }

  /** Launcher speed in motor RPM. */
  public double getLauncherRpm() {
    return intakeLauncherRoller.getEncoder().getVelocity();
  }

  /** The RPM the launcher is being held at, or 0 if it is not in velocity mode. */
  public double getLauncherTargetRpm() {
    return launcherTargetRpm;
  }

  /** True when the launcher is in velocity mode and within tolerance of its target speed. */
  public boolean atSpeed() {
    return launcherTargetRpm > 0.0
        && Math.abs(getLauncherRpm() - launcherTargetRpm) <= LAUNCHER_READY_TOLERANCE_RPM;
  }

  /** The voltage last sent to the feeder roller. */
  public double getFeederVoltage() {
    return feederVoltage;
  }

  // A method to stop the rollers
  public void stop() {
    launcherTargetRpm = 0.0;
    feederVoltage = 0.0;
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  /**
   * Releases the motor controllers. The robot never calls this; tests do, because REVLib
   * refuses a second SparkMax with the same CAN ID in one program.
   */
  public void close() {
    feederRoller.close();
    intakeLauncherRoller.close();
  }

  @Override
  public void periodic() {
    updateTunableGains();

    // Target vs actual speed: the graph to watch while tuning kV and kP.
    Logger.recordOutput("Launcher/TargetRpm", launcherTargetRpm);
    Logger.recordOutput("Launcher/ActualRpm", getLauncherRpm());
    Logger.recordOutput("Launcher/AtSpeed", atSpeed());
    Logger.recordOutput("Launcher/AppliedOutput", intakeLauncherRoller.getAppliedOutput());
    Logger.recordOutput("Launcher/FeederVoltage", feederVoltage);
  }

  /** Re-applies kP/kV when they change on the dashboard (only touches RAM, not flash). */
  private void updateTunableGains() {
    double newKp = SmartDashboard.getNumber("Launcher/kP", launcherKp);
    double newKv = SmartDashboard.getNumber("Launcher/kV", launcherKv);
    launcherKv = newKv;

    if (newKp != launcherKp) {
      launcherKp = newKp;
      SparkMaxConfig gains = new SparkMaxConfig();
      gains.closedLoop.p(launcherKp);
      intakeLauncherRoller.configure(
          gains, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  @Override
  public void simulationPeriodic() {
    // 1) What voltage is the code currently applying to the launcher?
    launcherPhysics.setInputVoltage(
        launcherSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // 2) Advance the physics by one 20ms step.
    launcherPhysics.update(0.02);

    // 3) Copy the result back into the simulated encoder (motor RPM, our encoder units).
    launcherSim.iterate(
        launcherPhysics.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);
  }
}
