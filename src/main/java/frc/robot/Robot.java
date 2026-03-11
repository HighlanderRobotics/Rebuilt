// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure.SuperState;
import frc.robot.components.cancoder.CANcoderIO;
import frc.robot.components.cancoder.CANcoderIOSim;
import frc.robot.components.candle.CANdleIOReal;
import frc.robot.components.pivot.PivotIO;
import frc.robot.components.pivot.PivotIOSim;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOSim;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.EmptyClimberSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.LindexerSubsystem;
import frc.robot.subsystems.indexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.FintakeSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.SlapdownSubsystem;
import frc.robot.subsystems.led.CANdleSubsystem;
import frc.robot.subsystems.led.LEDIOReal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.HoodIO;
import frc.robot.subsystems.shooter.HoodIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.TurretIO;
import frc.robot.subsystems.shooter.TurretIOSim;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.odometry.PhoenixOdometryThread;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.FieldUtils.ClimbTargets;
import frc.robot.utils.FieldUtils.TrenchPoses;
import frc.robot.utils.FuelSim;
import frc.robot.utils.autoaim.AutoAim;
import java.util.Arrays;
import java.util.Optional;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  /** This is whether or not the robot is real, sim, or replay */
  public enum RobotMode {
    REAL,
    SIM,
    REPLAY
  }

  /** This is whether or not the robot is Alpha or Comp */
  public enum RobotEdition {
    ALPHA,
    COMP
  }

  @AutoLogOutput(key = "Robot/Climb Target")
  private boolean leftClimbTarget = false; // TODO change to be operator controller

  public static final RobotMode ROBOT_MODE = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;
  // public static final RobotEdition ROBOT_EDITION = RobotEdition.COMP;
  public static final RobotEdition ROBOT_EDITION;
  public static final RobotEdition SIM_ROBOT_EDITION = RobotEdition.COMP;
  public static final RobotEdition REPLAY_ROBOT_EDITION = RobotEdition.ALPHA;
  private static final Alert unknownRioAlert =
      new Alert("!! Unknown Rio detected. Defaulting to comp", AlertType.kError);

  // for replay to work properly this needs to match the edition in the log
  static {
    switch (ROBOT_MODE) {
      case REAL:
        switch (RobotController.getSerialNumber()) {
          case "023D2BD2":
            ROBOT_EDITION = RobotEdition.ALPHA;
            break;
          case "0332054A":
            ROBOT_EDITION = RobotEdition.COMP;
            break;
          default:
            // defaulting to comp is probably safer?
            ROBOT_EDITION = RobotEdition.COMP;
            unknownRioAlert.set(true);
        }
        break;
      case SIM:
        // you're gonna have to just lock in on this
        ROBOT_EDITION = SIM_ROBOT_EDITION;
        break;
      case REPLAY:
        // you're gonna have to just lock in on this
        ROBOT_EDITION = REPLAY_ROBOT_EDITION;
        break;

      default:
        ROBOT_EDITION = RobotEdition.COMP;
    }
  }

  /**
   * This is for when we're testing shot and extension numbers and should be FALSE once bring up is
   * complete
   */
  public static final boolean TUNING_MODE = true;

  public boolean hasZeroedSinceStartup = false;

  // Add stuff related to dashboard alerts
  private final Alert driverJoystickDisconnectedAlert =
      new Alert("Driver controller disconnected!", AlertType.kError);
  private final Alert operatorJoystickDisconnectedAlert =
      new Alert("Operator controller disconnected!", AlertType.kError);
  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert canivoreErrorAlert =
      new Alert("CANivore errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.kWarning);
  private final Alert wrongRobotAlert =
      new Alert(
          "!! ALPHA CODE HAS BEEN DEPLOYED TO COMP BOT THIS IS VERY BAD !!", AlertType.kError);
  private final Alert tuningModeAlert =
      new Alert(
          "! Tuning mode is enabled while FMS attached, this is Quite bad !", AlertType.kWarning);

  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private final Timer disabledTimer = new Timer();
  private static final double CAN_ERROR_TIME_THRESHOLD = 0.5; // Seconds to disable alert
  private static final double CANIVORE_ERROR_TIME_THRESHOLD = 0.5;

  private static int lowBatteryCycleCount = 0;
  private static final double lowBatteryVoltage =
      12.1; // TODO 11.8 for practice batteries and 12.2 for comp batteries. maybe also do leds?
  private static final double lowBatteryDisabledTime = 1.5;
  private static final double lowBatteryMinCycleCount = 10;

  /**
   * As per the 2026+ ctre api we should be passing in the actual canbus object to any ctre device
   * constructors, NOT the name as a string
   */
  public static CANBus canivore = new CANBus("*");

  // Declare subsystems that aren't part of the superstructure
  // This means they can do stuff regardless of what state the robot's in
  // since none of them are dependent on the state - f.e. climber can do what it wants
  // Subsystems that *are* part of the superstructure are initialized later

  // swervesubsystem decides on its own whether or not to use alpha or comp swerve constants
  private final SwerveSubsystem swerve = new SwerveSubsystem(canivore);
  private final LEDSubsystem leds;
  private final ClimberSubsystem climber;

  private Intake intake = null;
  private Shooter shooter = null;
  private final CANdleSubsystem candle =
      new CANdleSubsystem(new CANdleIOReal(0, CANdleSubsystem.getCandleConfig(), canivore));

  private FuelSim fuelSim = new FuelSim();

  // climber only exists for the comp bot - this is accounted for later

  private final Superstructure superstructure;

  private final CommandXboxControllerSubsystem driver = new CommandXboxControllerSubsystem(0);
  private final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  // Assign non-superstructure triggers
  @AutoLogOutput(key = "Superstructure/Autoaim Request")
  private Trigger autoAimReq;

  // TODO
  //   @AutoLogOutput(key = "Superstructure/Autoaim Request")
  //   private Trigger climbAutoAlignInAutoReq;

  // Auto stuff
  private final Autos autos;
  private Optional<Alliance> lastAlliance = Optional.empty();
  @AutoLogOutput boolean haveAutosGenerated = false;
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autos");

  // temporarily override map with empty map to avoid collisions with reefscape elements
  // unfortunately this also turns off collisions with walls but that's fine
  // TODO update once rebuilt is added to maplesim
  private static class EvergreenArena extends SimulatedArena {
    protected EvergreenArena() {
      super(new FieldMap() {});
    }

    @Override
    public void placeGamePiecesOnField() {}
  }

  static {
    SimulatedArena.overrideInstance(new EvergreenArena());
  }

  // this is here because it doesn't like that the power distribution logger is never closed
  @SuppressWarnings("resource")
  public Robot() {

    // these have to be instantiated as null to ensure that there's always *something* in it
    // although it always gets assigned to the robot-specific version in the switch-case statement
    // below, the compiler doesn't technically know that for sure
    // that would be bad because if for some reason we had some case of ROBOT_EDITION that wasn't
    // accounted for, it wouldn't be able to pass anything to the superstructure below and it would
    // break
    // granted this would never actually happen but
    Indexer indexer = null;

    // this looks at the ROBOT_EDITION variable and decides which version of each subsystem to
    // create based on that
    // alpha: lindexer, fintake, shooter
    // comp: spindexer, lintake, turret
    switch (ROBOT_EDITION) {
      case ALPHA:
        indexer =
            new LindexerSubsystem(
                canivore,
                (ROBOT_MODE == RobotMode.REAL)
                    ? new RollerIO(9, LindexerSubsystem.getIndexerConfigs(), canivore)
                    : new RollerIOSim(
                        9,
                        LindexerSubsystem.getIndexerConfigs(),
                        new DCMotorSim(
                            LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX44Foc(1), 0.003, LindexerSubsystem.GEAR_RATIO),
                            DCMotor.getKrakenX44Foc(1)),
                        MotorType.KrakenX44,
                        canivore),
                (ROBOT_MODE == RobotMode.REAL)
                    ? new RollerIO(10, LindexerSubsystem.getKickerConfigs(), canivore)
                    : new RollerIOSim(
                        10,
                        LindexerSubsystem.getKickerConfigs(),
                        new DCMotorSim(
                            LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX44Foc(1),
                                0.00001,
                                LindexerSubsystem.KICKER_GEAR_RATIO),
                            DCMotor.getKrakenX44Foc(1)),
                        MotorType.KrakenX44,
                        canivore));

        shooter =
            new ShooterSubsystem(
                ROBOT_MODE == RobotMode.REAL
                    ? new HoodIO(ShooterSubsystem.getHoodConfig(), canivore, 11)
                    : new HoodIOSim(
                        canivore,
                        ShooterSubsystem.getHoodConfig(),
                        ShooterSubsystem.HOOD_GEAR_RATIO,
                        11,
                        ShooterSubsystem.HOOD_MIN_ROTATION,
                        ShooterSubsystem.HOOD_MAX_ROTATION),
                ROBOT_MODE == RobotMode.REAL
                    ? new FlywheelIO(ShooterSubsystem.getFlywheelConfig(), canivore, 12, 13)
                    : new FlywheelIOSim(
                        ShooterSubsystem.getFlywheelConfig(),
                        canivore,
                        ShooterSubsystem.FLYWHEEL_GEAR_RATIO,
                        11,
                        12));

        intake =
            new FintakeSubsystem(
                (ROBOT_MODE == RobotMode.REAL)
                    ? new RollerIO(8, FintakeSubsystem.getIntakeConfig(), canivore)
                    : new RollerIOSim(
                        8,
                        FintakeSubsystem.getIntakeConfig(),
                        new DCMotorSim(
                            LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX44Foc(1), 0.001, FintakeSubsystem.GEAR_RATIO),
                            DCMotor.getKrakenX44Foc(1)),
                        MotorType.KrakenX44,
                        canivore),
                canivore);
        // note that the climber is not instantiated here
        break;
      case COMP:
        indexer =
            new SpindexerSubsystem(
                canivore,
                (ROBOT_MODE == RobotMode.REAL)
                    ? new RollerIO(9, SpindexerSubsystem.getIndexerConfigs(), canivore)
                    : new RollerIOSim(
                        9,
                        SpindexerSubsystem.getIndexerConfigs(),
                        new DCMotorSim(
                            LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX44Foc(1),
                                0.003,
                                SpindexerSubsystem.SPINNER_GEAR_RATIO),
                            DCMotor.getKrakenX44Foc(1)),
                        MotorType.KrakenX44,
                        canivore),
                (ROBOT_MODE == RobotMode.REAL)
                    ? new RollerIO(10, SpindexerSubsystem.getKickerConfigs(), canivore)
                    : new RollerIOSim(
                        10,
                        SpindexerSubsystem.getKickerConfigs(),
                        new DCMotorSim(
                            LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX44Foc(1),
                                0.00001,
                                SpindexerSubsystem.KICKER_GEAR_RATIO),
                            DCMotor.getKrakenX44Foc(1)),
                        MotorType.KrakenX44,
                        canivore));
        intake =
            (ROBOT_MODE == RobotMode.REAL)
                ? new SlapdownSubsystem(
                    new PivotIO(
                        14,
                        SlapdownSubsystem.getPivotConfig(),
                        canivore), // Assuming same can id as extension
                    new CANcoderIO(6, SlapdownSubsystem.getCancoderConfig(), canivore),
                    new RollerIO(8, SlapdownSubsystem.getRollerConfig(), canivore))
                : new SlapdownSubsystem(
                    new PivotIOSim(
                        14,
                        SlapdownSubsystem.getPivotConfig(),
                        canivore,
                        new SingleJointedArmSim(
                            DCMotor.getKrakenX44Foc(1),
                            SlapdownSubsystem.PIVOT_GEAR_RATIO,
                            0.07,
                            Units.inchesToMeters(13.146739),
                            SlapdownSubsystem.PIVOT_MIN_POSITION.getRadians(),
                            SlapdownSubsystem.PIVOT_MAX_POSITION.getRadians(),
                            true,
                            SlapdownSubsystem.PIVOT_MIN_POSITION.getRadians()),
                        MotorType.KrakenX44,
                        SlapdownSubsystem.PIVOT_GEAR_RATIO),
                    new CANcoderIOSim(6, SlapdownSubsystem.getCancoderConfig(), canivore),
                    new RollerIOSim(
                        8,
                        SlapdownSubsystem.getRollerConfig(),
                        new DCMotorSim(
                            LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX44Foc(1),
                                0.01,
                                SlapdownSubsystem.ROLLER_GEAR_RATIO),
                            DCMotor.getKrakenX44Foc(1)),
                        MotorType.KrakenX44,
                        canivore));
        shooter =
            new TurretSubsystem(
                ROBOT_MODE == RobotMode.REAL
                    ? new FlywheelIO(TurretSubsystem.getFlywheelConfig(), canivore, 12, 13)
                    : new FlywheelIOSim(
                        TurretSubsystem.getFlywheelConfig(),
                        canivore,
                        TurretSubsystem.FLYWHEEL_GEAR_RATIO,
                        11,
                        12),
                ROBOT_MODE == RobotMode.REAL
                    ? new HoodIO(TurretSubsystem.getHoodConfig(), canivore, 11)
                    : new HoodIOSim(
                        canivore,
                        TurretSubsystem.getHoodConfig(),
                        TurretSubsystem.HOOD_GEAR_RATIO,
                        11,
                        TurretSubsystem.HOOD_MIN_ANGLE,
                        TurretSubsystem.HOOD_MAX_ANGLE),
                ROBOT_MODE == RobotMode.REAL ? new TurretIO(canivore) : new TurretIOSim(canivore),
                ROBOT_MODE == RobotMode.REAL
                    ? new CANcoderIO(5, TurretSubsystem.getCancoder24tConfigs(), canivore)
                    : new CANcoderIOSim(5, TurretSubsystem.getCancoder24tConfigs(), canivore),
                ROBOT_MODE == RobotMode.REAL
                    ? new CANcoderIO(4, TurretSubsystem.getCancoder26tConfigs(), canivore)
                    : new CANcoderIOSim(4, TurretSubsystem.getCancoder26tConfigs(), canivore),
                fuelSim);
        break;
    }
    climber =
        ROBOT_EDITION == RobotEdition.ALPHA
            ? new EmptyClimberSubsystem(canivore)
            : new ClimberSubsystem(
                new ClimberIO(canivore)); // TODO: SWITCH BACK TO REAL CLIMBER WHEN FIXED
    // : new ClimberSubsystem(new ClimberIO(canivore));
    // now that we've assigned the correct subsystems based on robot edition, we can pass them into
    // the superstructure
    superstructure =
        new Superstructure(swerve, indexer, intake, shooter, climber, driver, operator);
    addCompSysids(climber, indexer, intake, shooter);

    autoAimReq =
        driver
            .leftBumper()
            .or(
                new Trigger(
                        () ->
                            Superstructure.getState() == SuperState.SPIN_UP_SCORE
                                || Superstructure.getState() == SuperState.SCORE)
                    .and(() -> isTeleopEnabled()));

    //  climbAutoAlignInAutoReq = Autos.autoAlignClimbReq;

    DriverStation.silenceJoystickConnectionWarning(true);
    SignalLogger.enableAutoLogging(false);
    RobotController.setBrownoutVoltage(6.0);

    // Metadata about the current code running on the robot
    Logger.recordMetadata("Codebase", "Rebuilt");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("Robot Mode", ROBOT_MODE.toString());
    Logger.recordMetadata("Robot Edition", ROBOT_EDITION.toString());
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // log if we have uncommitted changes
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // set up logging stuff depending on robot mode
    switch (ROBOT_MODE) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        // TODO confirm pdp vs pdh
        // apparently LoggedPowerDistribution doesn't work with the pdp 2.0
        // LoggedPowerDistribution.getInstance(1, ModuleType.kCTRE); // Enables power distribution
        // logging
        new PowerDistribution(1, ModuleType.kCTRE);
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
    }
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

    Logger.recordOutput("Canivore Status", canivore.getStatus().Status);
    Logger.recordOutput("Robot Edition", ROBOT_EDITION);

    shooter.turretInit();
    intake.slapdownInit();

    PhoenixOdometryThread.getInstance().start();

    SmartDashboard.putData("[BE ENABLED] Current zero climber", climber.runCurrentZeroing());
    SmartDashboard.putData(
        "Zero Intake Off Cancoder", intake.zeroPivotOffCancoder().ignoringDisable(true));
    SmartDashboard.putData("Zero Hood", shooter.zeroHood().ignoringDisable(true));
    SmartDashboard.putData(
        "Test shot",
        Commands.parallel(
            shooter.torqueCurrentTest(swerve::getPose, swerve::getVelocityFieldRelative),
            Commands.waitUntil(new Trigger(shooter::atFlywheelVelocitySetpoint).debounce(0.1))
                .andThen(indexer.testShoot())));
    SmartDashboard.putData(
        "Set Turret to 0", shooter.resetTurretToPosition(Rotation2d.kZero).ignoringDisable(true));
    SmartDashboard.putData(
        "Rezero turret against cancoders",
        shooter
            .resetTurretToPosition(shooter.getCalculatedTurretRotations())
            .ignoringDisable(true));

    leds = new LEDSubsystem(new LEDIOReal()); // TODO sim
    candle.setDefaultCommand(candle.test().ignoringDisable(true));

    // Set default commands
    driver.setDefaultCommand(driver.rumbleCmd(0.0, 0.0));
    operator.setDefaultCommand(operator.rumbleCmd(0.0, 0.0));
    shooter.setDefaultCommand(shooter.rest(swerve::getPose, swerve::getVelocityFieldRelative));
    swerve.setDefaultCommand(
        swerve
            .driveOpenLoopFieldRelative(
                () ->
                    new ChassisSpeeds(
                            modifyJoystick(driver.getLeftY())
                                * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                            modifyJoystick(driver.getLeftX())
                                * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                            modifyJoystick(driver.getRightX())
                                * SwerveSubsystem.SWERVE_CONSTANTS.getMaxAngularSpeed())
                        .times(-1))
            .withName("default"));
    // swerve.setDefaultCommand(swerve.stop());
    indexer.setDefaultCommand(indexer.rest());
    intake.setDefaultCommand(intake.restExtended());
    climber.setDefaultCommand(climber.retract());
    // swerve.faceHubSOTM(
    //     () ->
    //         modifyJoystick(driver.getLeftX())
    //             * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
    //     () ->
    //         modifyJoystick(driver.getLeftY())
    //             * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed()
    //             * -1));

    addControllerBindings(indexer, shooter, intake);

    // Auto things
    autos = new Autos(swerve, climber);
    autoChooser.addDefaultOption("None", Commands.none());

    // Run auto when auto starts. Matches Choreolib's defer impl
    RobotModeTriggers.autonomous()
        .whileTrue(Commands.defer(() -> autoChooser.get().asProxy(), Set.of()));

    // Add autos on alliance change
    new Trigger(
            () -> {
              var allianceChanged = !DriverStation.getAlliance().equals(lastAlliance);
              lastAlliance = DriverStation.getAlliance();
              return allianceChanged && DriverStation.getAlliance().isPresent();
            })
        .onTrue(
            Commands.runOnce(() -> addAutos())
                .alongWith(
                    leds.blinkCmd(Color.kWhite, Color.kBlack, 20.0)
                        .withTimeout(1.0)
                        .ignoringDisable(true)));
    // TODO tbh idk if the leds will work here
    // Add autos when first connecting to DS
    new Trigger(
            () ->
                DriverStation.isDSAttached()
                    && DriverStation.getAlliance().isPresent()
                    && !haveAutosGenerated)
        .onTrue(Commands.print("connected"))
        .onTrue(
            Commands.runOnce(() -> addAutos())
                .alongWith(leds.blinkCmd(Color.kWhite, Color.kBlack, 20.0).withTimeout(1.0))
                .ignoringDisable(true));

    SmartDashboard.putData("Add autos", Commands.runOnce(this::addAutos).ignoringDisable(true));
    // SmartDashboard.putData("Zero hood", shooter.zeroHood().ignoringDisable(true));
    // SmartDashboard.putData("Test Shot", shooter.testShoot());

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    // log when commands get interrupted
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (interrupted, interrupting) -> {
              System.out.println("Interrupted: " + interrupted);
              System.out.println(
                  "Interrputing: "
                      + (interrupting.isPresent() ? interrupting.get().getName() : "none"));
            });

    fuelSim.spawnStartingFuel();
    fuelSim.registerIntake(
        Units.inchesToMeters(-14),
        Units.inchesToMeters(14),
        Units.inchesToMeters(14),
        Units.inchesToMeters(20) // robot-centric coordinates for bounding box in meters
        // (optional) BooleanSupplier for whether the intake should be active at a given moment
        ); // (optional) Runnable called whenever a fuel is intaked

    fuelSim.registerRobot(
        Units.inchesToMeters(28), // from left to right in meters
        Units.inchesToMeters(28), // from front to back in meters
        Units.inchesToMeters(4), // from floor to top of bumpers in meters
        swerve::getPose, // Supplier<Pose2d> of robot pose
        swerve
            ::getVelocityFieldRelative); // Supplier<ChassisSpeeds> of field-centric chassis speeds
    fuelSim.start();
  }

  /** Scales a joystick value for teleop driving */
  private static double modifyJoystick(double val) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(val, 2)) * Math.signum(val), 0.02);
  }

  private void addControllerBindings(Indexer indexer, Shooter shooter, Intake intake) {
    // heading reset
    driver
        .leftStick()
        .and(driver.rightStick())
        .onTrue(
            Commands.runOnce(
                () ->
                    swerve.setYaw(
                        DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)
                            // ? Rotation2d.kCW_90deg
                            // : Rotation2d.kCCW_90deg)));
                            ? Rotation2d.kZero
                            : Rotation2d.k180deg)));

    // autoaim (alpha)
    autoAimReq
        .and(() -> ROBOT_EDITION == RobotEdition.ALPHA)
        .whileTrue(
            // swerve.faceHubSOTM(
            //     () ->
            //         modifyJoystick(driver.getLeftY())
            //             * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
            //     () ->
            //         modifyJoystick(driver.getLeftX())
            //             * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed()));
            swerve.faceHub(
                () ->
                    -1
                        * modifyJoystick(driver.getLeftY())
                        * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                () ->
                    -1
                        * modifyJoystick(driver.getLeftX())
                        * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                ROBOT_EDITION == RobotEdition.ALPHA
                    ? AutoAim.ALPHA_HUB_SHOT_TREE
                    : AutoAim.COMP_HUB_SHOT_TREE));

    // climbAutoAlignInAutoReq.whileTrue(
    //     swerve.alignToClimb(
    //         () ->
    //             ClimbTargets.CLIMB_TARGETS_LIST.stream()
    //                 .filter(target -> target.getLeftHanded() == leftClimbTarget)
    //                 .filter(
    //                     target ->
    //                         target.isBlueAlliance()
    //                             == (DriverStation.getAlliance().orElse(Alliance.Blue)
    //                                 == Alliance.Blue))
    //                 .findFirst()
    //                 .get()));

    // climbAutoAlignInAutoReq.whileTrue(
    //     swerve.alignToClimb(
    //         () ->
    //             ClimbTargets.CLIMB_TARGETS_LIST.stream()
    //                 .filter(target -> target.getLeftHanded() == leftClimbTarget)
    //                 .filter(
    //                     target ->
    //                         target.isBlueAlliance()
    //                             == (DriverStation.getAlliance().orElse(Alliance.Blue)
    //                                 == Alliance.Blue))
    //                 .findFirst()
    //                 .get()));

    // new Trigger(swerve::isCloseToBump)
    //     .whileTrue(
    //         swerve.bumpAlign(
    //             () ->
    //                 DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)
    //                     ? -1
    //                     : 1
    //                         * modifyJoystick(driver.getLeftY())
    //                         * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
    //             () ->
    //                 DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)
    //                     ? -1
    //                     : 1
    //                         * modifyJoystick(driver.getLeftX())
    //                         * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed()));

    new Trigger(swerve::isNearTrench)
        .and(() -> Superstructure.getState() != SuperState.INTAKE)
        .and(() -> isTeleopEnabled())
        .whileTrue(
            swerve
                // .driveClosedLoopFieldRelative(
                //     () ->
                //         new ChassisSpeeds(
                //                 modifyJoystick(driver.getLeftY())
                //                     * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                //                 modifyJoystick(driver.getLeftX())
                //                     * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                //                 AutoAlign.calculateRotationVelocity(
                //                     swerve.getRotation(), Rotation2d.fromDegrees(30)))
                //             .times(-1))
                .trenchAlign(
                    () ->
                        modifyJoystick(driver.getLeftY())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                    () ->
                        modifyJoystick(driver.getLeftX())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed())
                .alongWith(Commands.print("afkljsdflkjs")));
    // 0));

    // current zero shooter hood
    driver
        .b()
        .whileTrue(
            shooter
                .resetTurretToPosition(shooter.getCalculatedTurretRotations())
                .andThen(
                    Commands.parallel(shooter.runCurrentZeroing(), intake.runCurrentZeroing())));

    new Trigger(() -> AutoAim.targetInTurretDeadzone())
        .onTrue(driver.rumbleCmd(1, 1).withTimeout(0.25));

    operator
        .leftBumper()
        .or(Autos.autoLeftClimbReq)
        .onTrue(Commands.runOnce(() -> leftClimbTarget = true));
    operator
        .rightBumper()
        .or(Autos.autoLeftClimbReq.negate())
        .onTrue(Commands.runOnce(() -> leftClimbTarget = false));

    // TODO: ACTUAL BINDING LOL
    // test shot
    driver
        .rightBumper()
        .whileTrue(
            //         Commands.parallel(
            //             // shooter.torqueCurrentTest(),
            //             shooter.testShoot(swerve::getPose, swerve::getVelocityFieldRelative),
            //             Commands.waitUntil(
            //                     new Trigger(shooter::atFlywheelVelocitySetpoint).debounce(0.05)
            //                     // .and(shooter::atTurretSetpoint)
            //                     // .debounce(0.25)
            //                     )
            //                 .andThen(indexer.kick())));
            // SmartDashboard.putData(
            //     "climb align",
            swerve.alignToClimb(
                () ->
                    ClimbTargets.CLIMB_TARGETS_LIST.stream()
                        .filter(target -> target.getLeftHanded() == leftClimbTarget)
                        .filter(
                            target ->
                                target.isBlueAlliance()
                                    == (DriverStation.getAlliance().orElse(Alliance.Blue)
                                        == Alliance.Blue))
                        .findFirst()
                        .get()));
    // turn swerve if target is in turret deadzone
    driver
        .leftBumper()
        .and(AutoAim::targetInTurretDeadzone)
        .and(() -> Superstructure.getState().isAScoreState())
        .whileTrue(
            swerve.faceHubComp(
                () ->
                    -1
                        * modifyJoystick(driver.getLeftY())
                        * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                () ->
                    -1
                        * modifyJoystick(driver.getLeftX())
                        * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                shooter::getTurretPosition));
    // ---zeroing stuff---

    // create triggers for joystick disconnect alerts
    new Trigger(() -> DriverStation.isJoystickConnected(0))
        .negate()
        .onTrue(Commands.runOnce(() -> driverJoystickDisconnectedAlert.set(true)))
        .onFalse(Commands.runOnce(() -> driverJoystickDisconnectedAlert.set(false)));

    new Trigger(() -> DriverStation.isJoystickConnected(1))
        .negate()
        .onTrue(Commands.runOnce(() -> operatorJoystickDisconnectedAlert.set(true)))
        .onFalse(Commands.runOnce(() -> operatorJoystickDisconnectedAlert.set(false)));
  }

  private void addAutos() {
    System.out.println("------- Regenerating Autos");
    System.out.println(
        "Regenerating Autos on " + DriverStation.getAlliance().map((a) -> a.toString()));
    autoChooser.addOption("Depot Feed Climb", autos.getDepotFeedClimbAuto());
    autoChooser.addOption("Depot Score Climb", autos.getDepotScoreClimbAuto());
    autoChooser.addOption("Outpost Feed Climb", autos.getOutpostFeedClimbAuto());
    autoChooser.addOption("Outpost Score Climb", autos.getOutpostScoreClimbAuto());
    autoChooser.addOption("Fill Depot Score Climb", autos.getFillDepotScoreClimbAuto());
    autoChooser.addOption("Fill Outpost Score Climb", autos.getFillOutpostScoreClimbAuto());
    autoChooser.addOption("Depot Climb", autos.getDepotClimbAuto());
    autoChooser.addOption("Depot Outpost Climb", autos.getDepotOutpostClimbAuto());
    autoChooser.addOption("Test Auto", autos.getTestAuto());

    haveAutosGenerated = true;
    System.out.println("Done generating autos");
  }

  // Sysid Autos
  private void addCompSysids(
      ClimberSubsystem climber, Indexer indexer, Intake intake, Shooter shooter) {
    autoChooser.addOption("Climber Sysid", climber.runClimberSysid());
    autoChooser.addOption("Indexer Roller Sysid", indexer.runRollerSysId());
    autoChooser.addOption("Intake Roller Sysid", intake.runRollerSysid());
    autoChooser.addOption("Intake Pivot Sysid", intake.runPivotSysid());
    autoChooser.addOption("Flywheel Sysid", shooter.runFlywheelSysid());

    autoChooser.addOption("Hood Sysid", shooter.runHoodSysid());
    autoChooser.addOption("Turret Sysid", shooter.runTurretSysid());
    autoChooser.addOption("Kicker Sysid", indexer.runKickerSysId());
    autoChooser.addOption("Turn Sysid", swerve.runTurnSysid());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    superstructure.periodic();

    // TODO: YAW VALUE FROM HARDWARE
    Pose3d turretPose =
        new Pose3d(
            new Translation3d(-0.177413, -0.111702, 0.350341),
            // new Rotation3d(0, 0, Units.degreesToRadians(turretAngle.getAsDouble())));
            new Rotation3d(0, 0, shooter.getTurretPosition().getRadians()));
    // 0));
    // ));
    // TODO: USE MEASURED EXTENSIONS AND ANGLES
    Logger.recordOutput(
        "Robot/Mechanism Poses",
        new Pose3d[] {
          // Turret
          turretPose,
          // Hood
          turretPose
              // First transform the hood out to the hood pivot, and rotate by the amount needed
              .transformBy(
                  new Transform3d(
                      new Translation3d(-0.095638, 0, 0.095123),
                      new Rotation3d(0, shooter.getHoodPosition().getRadians() * 1, 0)))
              // Then, transform the hood back to the correct location relative to the turret
              .transformBy(
                  new Transform3d(
                      new Translation3d(-0.095638, 0, 0.095123).times(-1), Rotation3d.kZero)),
          // Intake
          //   new Pose3d(
          //       intake.getPosition() * LintakeSubsystem.INTAKE_ROTATION.getCos(),
          //       0,
          //       -(intake.getPosition() * LintakeSubsystem.INTAKE_ROTATION.getSin()),
          //       Rotation3d.kZero),
          // Climber
          new Pose3d(0, 0, climber.getClimberExtensionMeters(), Rotation3d.kZero)
        });

    // TODO: ACTUAL SETPOINT
    Pose3d turretSetpoint =
        new Pose3d(
            new Translation3d(-0.177413, -0.111702, 0.350341),
            new Rotation3d(0, 0, shooter.getTurretSetpoint().getRadians()));
    // TODO: ACTUAL SETPOINTS
    Logger.recordOutput(
        "Robot/Mechanism Setpoints",
        new Pose3d[] {
          // Turret
          turretSetpoint,
          // Hood
          turretSetpoint
              // First transform the hood out to the hood pivot, and rotate by the amount needed
              .transformBy(
                  new Transform3d(
                      new Translation3d(-0.095638, 0, 0.095123),
                      new Rotation3d(0, shooter.getHoodSetpoint().getRadians() * -1, 0)))
              // Then, transform the hood back to the correct location relative to the turret
              .transformBy(
                  new Transform3d(
                      new Translation3d(-0.095638, 0, 0.095123).times(-1), Rotation3d.kZero)),
          // Intake
          //   new Pose3d(
          //       intake.getPositionSetpoint() * LintakeSubsystem.INTAKE_ROTATION.getCos(),
          //       0,
          //       -(intake.getPositionSetpoint() * LintakeSubsystem.INTAKE_ROTATION.getSin()),
          //       Rotation3d.kZero),
          // Climber
          new Pose3d(0, 0, climber.getClimberSetpointMeters(), Rotation3d.kZero)
        });

    updateAlerts();

    // Log climb poses
    Logger.recordOutput(
        "AutoAlign/Climb Targets",
        Arrays.stream(ClimbTargets.values())
            .map(target -> target.getPose())
            .toArray(Pose2d[]::new));

    Logger.recordOutput(
        "trench poses",
        Arrays.stream(TrenchPoses.values()).map(target -> target.getPose()).toArray(Pose2d[]::new));

    Logger.recordOutput("Turret/out of range", AutoAim.targetInTurretDeadzone());
  }

  public void updateAlerts() {

    if (ROBOT_EDITION == RobotEdition.ALPHA && DriverStation.isFMSAttached()) {
      wrongRobotAlert.set(true);
      // TODO make leds strobe red or something
    }
    if (TUNING_MODE && DriverStation.isFMSAttached()) tuningModeAlert.set(true);
    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(CAN_ERROR_TIME_THRESHOLD)
            && !canInitialErrorTimer.hasElapsed(CAN_ERROR_TIME_THRESHOLD));

    // Log CANivore status
    if (Robot.isReal()) {
      var canivoreStatus =
          Optional.of(canivore.getStatus()); // TODO i don't know if i'm doing the optionaling right
      if (canivoreStatus.isPresent()) {
        Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.get().Status.getName());
        Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.get().BusUtilization);
        Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.get().BusOffCount);
        Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.get().TxFullCount);
        Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.get().REC);
        Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.get().TEC);

        if (!canivoreStatus.get().Status.isOK()
            || canStatus.transmitErrorCount > 0
            || canStatus.receiveErrorCount > 0) {
          canivoreErrorTimer.restart();
        }
      }
      // TODO i don't really like how this doesn't seem to be sticky
      canivoreErrorAlert.set(
          !canivoreErrorTimer.hasElapsed(CANIVORE_ERROR_TIME_THRESHOLD)
              && !canInitialErrorTimer.hasElapsed(CAN_ERROR_TIME_THRESHOLD));
    }

    // Low battery alert
    lowBatteryCycleCount += 1;
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)
        && lowBatteryCycleCount >= lowBatteryMinCycleCount) {
      lowBatteryAlert.set(true);
    }
  }

  @Override
  public void simulationInit() {
    // Sets the odometry pose to start at the same place as maple sim pose
    swerve.resetMapleSimPose();
  }

  @Override
  public void simulationPeriodic() {
    fuelSim.updateSim();
    // Log zeroed poses for mechs and robot for debugging in sim
    Logger.recordOutput(
        "Robot/Zeroed Mechanism Poses",
        new Pose3d[] {
          // Turret
          new Pose3d(),
          // Hood
          new Pose3d(),
          new Pose3d(),
          new Pose3d()
        });

    Logger.recordOutput("Robot/Zero Position", new Pose2d());
  }

  @Override
  public void disabledInit() {
    addAutos();
    System.out.println("--------------Robot Disabled-----------");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    intake.slapdownInit();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    autos.setAllReqsFalsenotcmd();
  }

  @Override
  public void teleopInit() {
    intake.slapdownInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
