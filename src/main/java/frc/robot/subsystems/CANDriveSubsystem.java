package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
//import com.pathplanner.lib.controllers.PRAMamRamController;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The drivetrain: four NEO motors (two per side) on a classic tank ("differential") drive.
 *
 * <p>A SUBSYSTEM is one physical mechanism of the robot plus the code that talks to its
 * hardware. Subsystems are the NOUNS of your robot ("the drivetrain", "the arm"). Commands
 * (see the {@code commands} folder) are the VERBS that tell subsystems what to do. Only this
 * file is allowed to touch the drive motors; everything else must go through the public
 * methods below. That rule is what keeps robot code from turning into spaghetti.
 *
 * <p>What this subsystem provides:
 *
 * <ul>
 *   <li>{@link #arcadeDrive}/{@link #tankDrive} - make the robot move
 *   <li>Odometry - a live estimate of where the robot is on the field, computed from the
 *       wheel encoders + gyro every 20ms, viewable on the AdvantageScope/Glass field
 *   <li>Logging - every interesting number is recorded via AdvantageKit (see {@code
 *       docs/05-logging-and-dashboards.md})
 *   <li>Simulation - realistic physics when you run "Simulate Robot Code", so you can test
 *       without a robot (see {@code docs/06-simulation.md})
 * </ul>
 */
public class CANDriveSubsystem extends SubsystemBase {

  // ----------------------------------------------------------------------------------------
  // Hardware
  // ----------------------------------------------------------------------------------------

  // Each side has a "leader" the code talks to and a "follower" that automatically copies
  // the leader's output inside the motor controller firmware. This way the rest of the code
  // only ever thinks about two motors.
  private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_CAN_ID, MotorType.kBrushless);
  private final SparkMax leftFollower =
      new SparkMax(LEFT_FOLLOWER_CAN_ID, MotorType.kBrushless);
  private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_CAN_ID, MotorType.kBrushless);
  private final SparkMax rightFollower =
      new SparkMax(RIGHT_FOLLOWER_CAN_ID, MotorType.kBrushless);

  // The NEO's built-in encoder, read through the SPARK MAX. Thanks to the conversion factors
  // configured below, these report METERS and METERS/SECOND at the wheel, not motor turns.
  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  // The gyro. Knowing the robot's heading is what makes odometry accurate (encoders alone
  // can't tell the difference between driving straight and arcing).
  private final Pigeon2 pigeon = new Pigeon2(PIGEON_CAN_ID);

  // Phoenix 6 devices hand you cached "status signals" instead of live values. We grab the
  // two we care about once, then refresh them at the top of every periodic(). That's the
  // pattern CTRE recommends, and it guarantees the rest of the loop sees current data.
  private final StatusSignal<Angle> yawSignal = pigeon.getYaw();
  private final StatusSignal<AngularVelocity> yawRateSignal = pigeon.getAngularVelocityZWorld();

  // WPILib helper that turns (forward, rotation) or (left, right) requests into correctly
  // scaled outputs for the two sides. It also includes a motor-safety watchdog: if no drive
  // method is called for 0.1s while enabled, it stops the motors and warns. That behavior
  // is intentional, and it's why commands must keep calling arcadeDrive() every loop.
  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
  // ----------------------------------------------------------------------------------------
  // Odometry - "where am I on the field?"
  // ----------------------------------------------------------------------------------------

  // Combines gyro heading + how far each wheel has rolled into an (x, y, angle) pose,
  // updated every loop in periodic(). Field coordinates: x = away from your alliance wall,
  // y = to the left, angle = counter-clockwise positive. Units are meters and degrees.
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(Rotation2d.kZero, 0.0, 0.0);

  // A widget that draws the robot on a top-down field view in Glass / Elastic / the sim GUI.
  private final Field2d field = new Field2d();

  // ----------------------------------------------------------------------------------------
  // Simulation objects. These are only *used* when running on your laptop (see
  // simulationPeriodic below); on the real robot they simply sit idle. The physics sim
  // "backfills" the encoders and gyro, so odometry and logging behave exactly like real life.
  // ----------------------------------------------------------------------------------------

  private final DifferentialDrivetrainSim driveSim =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(2), // two NEOs per side
          GEAR_RATIO,
          ROBOT_MOI_KG_M2,
          ROBOT_MASS_KG,
          WHEEL_DIAMETER_METERS / 2.0, // radius
          TRACK_WIDTH_METERS,
          null); // no measurement noise, to keep sim results easy to reason about

  private final SparkMaxSim leftLeaderSim = new SparkMaxSim(leftLeader, DCMotor.getNEO(2));
  private final SparkMaxSim rightLeaderSim = new SparkMaxSim(rightLeader, DCMotor.getNEO(2));
  private final Pigeon2SimState pigeonSim = pigeon.getSimState();

  /** Configures all the hardware. Runs once when the robot code starts. */
  public CANDriveSubsystem() {
    // --- Configure the four SPARK MAXes ---------------------------------------------------
    // With REVLib you build a config object, then push it to the controller. We push with
    // kResetSafeParameters (start from known defaults, so a controller swapped in from
    // another robot doesn't keep surprise settings) and kPersistParameters (settings survive
    // a power cycle).
    SparkMaxConfig leftConfig = baseConfig(LEFT_INVERTED);
    leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = baseConfig(RIGHT_INVERTED);
    rightLeader.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Followers: same limits, plus "copy whatever your leader does". A follower copies the
    // leader's ACTUAL output (after inversion), so it does not need its own inverted flag -
    // both motors on a gearbox physically spin the same way.
    SparkMaxConfig leftFollowerConfig = baseConfig(false);
    leftFollowerConfig.follow(leftLeader);
    leftFollower.configure(
        leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightFollowerConfig = baseConfig(false);
    rightFollowerConfig.follow(rightLeader);
    rightFollower.configure(
        rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- Gyro ------------------------------------------------------------------------------
    pigeon.setYaw(0.0); // define "the way the robot is facing at power-on" as 0 degrees

    // DifferentialDrive applies its own tiny safety deadband; our real deadband is applied
    // to the joystick in TeleopDriveCommand, so this one can stay at its default.

    // Show the field widget on dashboards (Glass, Elastic, and the sim GUI all find it).
    SmartDashboard.putData("Field", field);
  }

  /** The settings shared by all four motor controllers. */
  private SparkMaxConfig baseConfig(boolean inverted) {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(inverted)
        .idleMode(IdleMode.kBrake) // brake = robot stops when sticks released (predictable)
        .smartCurrentLimit(CURRENT_LIMIT_AMPS) // protects breakers + battery, see Constants
        .voltageCompensation(NOMINAL_VOLTAGE); // consistent feel as the battery drains
    config
        .encoder
        .positionConversionFactor(POSITION_CONVERSION_FACTOR) // motor turns -> meters
        .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR); // RPM -> meters/second
    return config;
  }

  // ----------------------------------------------------------------------------------------
  // Driving - the public API commands use to make the robot move
  // ----------------------------------------------------------------------------------------

  /**
   * Drive "video game style": one input for speed, one for turning.
   *
   * @param forward +1.0 = full speed forward, -1.0 = full speed backward
   * @param rotation +1.0 = spin counter-clockwise (left), -1.0 = clockwise (right)
   */
  public void driveArcade(double forward, double rotation) {
    // 'false' = don't square the inputs here; TeleopDriveCommand shapes the joystick values
    // itself so that all the "driver feel" math lives in one visible place.
    drive.arcadeDrive(forward, rotation, false);
  }

  /**
   * Drive "true tank style": one input per side. Not used by default, but some drivers
   * prefer it. See docs/04-driving-and-controls.md for how to switch.
   */
  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right, false);
  }

  /** Stop all drive motors. Commands call this when they end so nothing keeps moving. */
  public void stop() {
    drive.stopMotor();
  }

  // ----------------------------------------------------------------------------------------
  // Sensors + odometry
  // ----------------------------------------------------------------------------------------

  /**
   * Where the robot thinks it is on the field (meters + heading). The @AutoLogOutput
   * annotation means AdvantageKit records this automatically every loop. Open the log in
   * AdvantageScope's field view and you can watch the robot drive around.
   */
  @AutoLogOutput(key = "Drive/Pose")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Robot heading in degrees, counter-clockwise positive, from odometry. */
  @AutoLogOutput(key = "Drive/HeadingDegrees")
  public double getHeadingDegrees() {
    return getPose().getRotation().getDegrees();
  }

  /** Average of both sides' distance traveled since the last pose reset, in meters. */
  public double getAverageDistanceMeters() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
  }

// 2. Method implementation:
  public ChassisSpeeds getRobotRelativeSpeeds() {
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
      leftEncoder.getVelocity(),
      rightEncoder.getVelocity()
    );
    return m_kinematics.toChassisSpeeds(wheelSpeeds);
  }
  
  public void driveRobotRelative(ChassisSpeeds speeds) {
  // Call toWheelSpeeds on kinematics, NOT drive
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

  // If you use feedforward/volts or closed-loop speed control, apply it here.
  // For basic voltage/tank drive:
    tankDrive(
      wheelSpeeds.leftMetersPerSecond, 
      wheelSpeeds.rightMetersPerSecond
    );
  }

  /** Turn rate in degrees per second (used by TurnToAngleCommand's tolerance check). */
  public double getTurnRateDegreesPerSec() {
    return yawRateSignal.getValueAsDouble();
  }

  /** The gyro's yaw as a Rotation2d, from the signal refreshed at the top of periodic(). */
  private Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(yawSignal.getValueAsDouble());
  }

  /**
   * Tell odometry "the robot is actually HERE". Called at the start of autonomous (so the
   * field origin is wherever you placed the robot) and by the driver's reset button. Note:
   * this does not move the robot; it just moves the math.
   */
  public void resetOdometry(Pose2d pose) {
    // The odometry class stores the current encoder/gyro readings as offsets, so we don't
    // need to zero the physical sensors.
    odometry.resetPosition(
        getGyroRotation(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
  }

  // ----------------------------------------------------------------------------------------
  // periodic() - runs automatically every 20ms, whether the robot is enabled or not.
  // Keep it to bookkeeping (updating odometry, dashboards, logs). Movement belongs in
  // commands, never here.
  // ----------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // Pull the newest gyro data into our cached signals (see the field comment above).
    BaseStatusSignal.refreshAll(yawSignal, yawRateSignal);

    // Fuse the latest sensor readings into the pose estimate.
    odometry.update(
        getGyroRotation(), leftEncoder.getPosition(), rightEncoder.getPosition());

    // Update the dashboard field drawing.
    field.setRobotPose(getPose());

    // --- Logging --------------------------------------------------------------------------
    // Everything recorded here (plus the @AutoLogOutput getters above) is sent to the
    // dashboard live AND saved into the .wpilog file. When something acts up at a
    // competition, this data is how you find out what actually happened.
    Logger.recordOutput("Drive/LeftPositionMeters", leftEncoder.getPosition());
    Logger.recordOutput("Drive/RightPositionMeters", rightEncoder.getPosition());
    Logger.recordOutput("Drive/LeftVelocityMetersPerSec", leftEncoder.getVelocity());
    Logger.recordOutput("Drive/RightVelocityMetersPerSec", rightEncoder.getVelocity());
    Logger.recordOutput("Drive/GyroYawDegrees", yawSignal.getValueAsDouble());
    // What the controllers are actually outputting (-1 to 1). If the robot "won't drive",
    // this tells you whether the code even asked it to.
    Logger.recordOutput("Drive/AppliedOutputLeft", leftLeader.getAppliedOutput());
    Logger.recordOutput("Drive/AppliedOutputRight", rightLeader.getAppliedOutput());
    // Motor currents as one array [leftLeader, leftFollower, rightLeader, rightFollower].
    // A motor drawing far more than its siblings usually means a mechanical problem.
    Logger.recordOutput(
        "Drive/CurrentAmps",
        new double[] {
          leftLeader.getOutputCurrent(),
          leftFollower.getOutputCurrent(),
          rightLeader.getOutputCurrent(),
          rightFollower.getOutputCurrent()
        });
  }

  // ----------------------------------------------------------------------------------------
  // simulationPeriodic() - like periodic(), but ONLY runs in simulation, right after it.
  // The pattern: feed the motor outputs into a physics model, then write the physics
  // results back into the simulated sensors. The rest of the code can't tell the difference.
  // ----------------------------------------------------------------------------------------
  @Override
  public void simulationPeriodic() {
    // 1) What voltage is the code currently applying to each side?
    driveSim.setInputs(
        leftLeaderSim.getAppliedOutput() * RobotController.getBatteryVoltage(),
        rightLeaderSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // 2) Advance the physics by one 20ms step.
    driveSim.update(0.02);

    // 3) Copy the physics results back into the "sensors". iterate() updates the simulated
    //    encoder position/velocity from the mechanism velocity we hand it (m/s, matching our
    //    conversion factors).
    leftLeaderSim.iterate(
        driveSim.getLeftVelocityMetersPerSecond(), RobotController.getBatteryVoltage(), 0.02);
    rightLeaderSim.iterate(
        driveSim.getRightVelocityMetersPerSecond(), RobotController.getBatteryVoltage(), 0.02);

    pigeonSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    pigeonSim.setRawYaw(driveSim.getHeading().getDegrees());

    // 4) Simulate battery sag: driving hard pulls the simulated battery voltage down, just
    //    like a real match. (Watch "BatteryVoltage" dip in AdvantageScope while you drive.)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(driveSim.getCurrentDrawAmps()));

    // Also log the "true" simulated pose. Comparing it against Drive/Pose shows you how
    // much odometry drifts, which is a great thing for students to explore.
    Logger.recordOutput("Drive/SimTruePose", driveSim.getPose());

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }
}
