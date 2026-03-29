// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.RobotEdition;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.FeedTargets;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.NewAutoAim;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  /** We should have a state for every single action the robot will perform. */
  public enum SuperState {
    IDLE,
    INTAKE,
    SPIN_UP_FEED,
    FEED,
    SPIN_UP_FEED_FLOW,
    FEED_FLOW,
    SPIN_UP_SCORE,
    SCORE,
    SPIN_UP_SCORE_FLOW,
    SCORE_FLOW,
    SPIT,
    PRE_CLIMB,
    CLIMB,
    POST_CLIMB,
    SPIN_UP_SCORE_PRE_CLIMB,
    SCORE_PRE_CLIMB,
    DEFENSE;
    public final Trigger trigger;

    private SuperState() {
      trigger = new Trigger(() -> state == this);
    }

    public Trigger getTrigger() {
      return trigger;
    }

    public boolean isAScoreState() {
      return this == SCORE
          || this == SPIN_UP_SCORE
          || this == SPIN_UP_SCORE_FLOW
          || this == SCORE_FLOW;
    }

    public boolean isAFeedState() {
      return this == FEED || this == SPIN_UP_FEED || this == SPIN_UP_FEED_FLOW || this == FEED_FLOW;
    }

    public boolean isAnIntakeState() {
      return this == INTAKE
          || this == SPIN_UP_FEED_FLOW
          || this == FEED_FLOW
          || this == SPIN_UP_SCORE_FLOW
          || this == SCORE_FLOW;
    }

    public boolean isAFlowState() {
      return this == FEED_FLOW
          || this == SCORE_FLOW
          || this == SPIN_UP_FEED_FLOW
          || this == SPIN_UP_SCORE_FLOW;
    }
  }

  public enum ShotTarget {
    SCORE,
    FEED
  }

  // left and right from driver pov
  public enum FeedTarget {
    LEFT,
    RIGHT
  }

  public enum FixedShotTarget {
    LEFT,
    MID,
    RIGHT,
    NONE
  }

  @AutoLogOutput(key = "Superstructure/State")
  private static SuperState state = SuperState.IDLE;

  private SuperState prevState = SuperState.IDLE;

  private Timer stateTimer = new Timer();

  private double getFPGATimestamp() {
    return Timer.getFPGATimestamp();
  }

  @AutoLogOutput(key = "Superstructure/match starttime")
  public static double matchStartTime;

  private double getTimeElapsed() {
    return getFPGATimestamp() - matchStartTime;
  }

  private double timeLeftinMatch() {
    return 140.00 - getTimeElapsed();
  }

  @AutoLogOutput(key = "Superstructure/Shift Timer")
  private double getTimeStampLeftInShift() {
    return getTimeLeftInShift();
  }

  @AutoLogOutput(key = "Superstructure/Current Shift")
  private String getCurrentShiftName() {
    return getCurrentShift();
  }

  @AutoLogOutput(key = "Scoring/Scoring Active")
  public boolean isScoringActive() {
    return isOurShift();
  }

  private final SwerveSubsystem swerve;
  private final Indexer indexer;
  private final Intake intake;
  private final Shooter shooter;
  private final ClimberSubsystem climber;
  private final CommandXboxControllerSubsystem driver;
  private final CommandXboxControllerSubsystem operator;

  // Declare triggers
  @AutoLogOutput(key = "Superstructure/Shoot Request")
  private Trigger shootReq;

  @AutoLogOutput(key = "Superstructure/Intake Request")
  private Trigger intakeReq;

  @AutoLogOutput(key = "Superstructure/Anti Jam Req")
  private Trigger antiJamReq;

  @AutoLogOutput(key = "Superstructure/Pre Climb Req")
  private Trigger preClimbReq;

  @AutoLogOutput(key = "Superstructure/Climb Req")
  private Trigger climbReq;

  @AutoLogOutput(key = "Superstructure/Shot Target")
  private static ShotTarget shotTarget = ShotTarget.SCORE;

  @AutoLogOutput(key = "Superstructure/Score Request")
  private Trigger scoreReq =
      new Trigger(() -> shotTarget == ShotTarget.SCORE)
          // .and(() -> canScore())
          .or(Autos.autoScoreReq);

  @AutoLogOutput(key = "Superstructure/Feed Request")
  private Trigger feedReq = new Trigger(() -> shotTarget == ShotTarget.FEED);

  @AutoLogOutput(key = "Superstructure/Can Score")
  private boolean canScore = canScore();

  private boolean flowState = false;

  @AutoLogOutput(key = "Superstructure/Flow State Request")
  private Trigger flowReq = new Trigger(() -> flowState);

  @AutoLogOutput(key = "Superstructure/Feed Target")
  private static FeedTarget feedTarget = FeedTarget.RIGHT;

  // spun up + hood at setpoint + pointing at target
  @AutoLogOutput(key = "Superstructure/Ready?")
  private Trigger readyTrigger;

  @AutoLogOutput(key = "Superstructure/Operator Pose Override?")
  private static boolean poseOverride = false;

  @AutoLogOutput(key = "Superstructure/Defense?")
  private boolean defense = false;

  @AutoLogOutput(key = "Superstructure/Defense Req")
  private Trigger defenseReq = new Trigger(() -> defense).or(Autos.autoDefenseReq);

  // @AutoLogOutput(key = "Superstructure/Fixed Shot")
  // private static FixedShotTarget fixedShotTarget = FixedShotTarget.NONE;

  /** Creates a new Superstructure. */
  public Superstructure(
      SwerveSubsystem swerve,
      Indexer indexer,
      Intake intake,
      Shooter shooter,
      ClimberSubsystem climber,
      CommandXboxControllerSubsystem driver,
      CommandXboxControllerSubsystem operator) {
    this.swerve = swerve;
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
    this.driver = driver;
    this.operator = operator;
    this.climber = climber;

    addTriggers();
    addTransitions();
    addCommands();

    stateTimer.start();
  }

  private void addTriggers() {
    // Toggles for feeding
    operator.x().onTrue(Commands.runOnce(() -> shotTarget = ShotTarget.SCORE));
    operator.y().onTrue(Commands.runOnce(() -> shotTarget = ShotTarget.FEED));

    operator.povUp().onTrue(Commands.runOnce(() -> defense = true));
    operator.povDown().onTrue(Commands.runOnce(() -> defense = false));

    // toggle for flow state
    operator
        .a()
        .and(DriverStation::isTeleop)
        .or(Autos.autoFlowReq)
        .onTrue(Commands.runOnce(() -> flowState = true));
    operator
        .b()
        .and(DriverStation::isTeleop)
        .or(Autos.autoFlowReq.negate().and(DriverStation::isAutonomous))
        .onTrue(Commands.runOnce(() -> flowState = false));

    operator.leftBumper().onTrue(Commands.runOnce(() -> feedTarget = FeedTarget.LEFT));
    operator.rightBumper().onTrue(Commands.runOnce(() -> feedTarget = FeedTarget.RIGHT));

    // operator.leftTrigger().onTrue(Commands.runOnce(() -> poseOverride = true));
    // operator.rightTrigger().onTrue(Commands.runOnce(() -> poseOverride = false));

    // operator
    //     .povLeft()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               fixedShotTarget = FixedShotTarget.LEFT;
    //               poseOverride = true;
    //             }));
    // operator
    //     .povUp()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               fixedShotTarget = FixedShotTarget.MID;
    //               poseOverride = true;
    //             }));
    // operator
    //     .povRight()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               fixedShotTarget = FixedShotTarget.RIGHT;
    //               poseOverride = true;
    //             }));
    // operator
    //     .povDown()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               fixedShotTarget = FixedShotTarget.NONE;
    //             }));

    operator.povUp().onTrue(Commands.parallel(intake.restRetracted(), shooter.stopTurret()));
    operator.povDown().onTrue(Commands.parallel(intake.restRetracted(), shooter.stopTurret()));
    shootReq =
        driver
            .rightTrigger()
            .and(DriverStation::isTeleop)
            .and(() -> canShoot())
            .or(Autos.autoScoreReq)
            .and(() -> canShoot()); // Maybe should include if its our turn? //TODO fix auto
    // bindings

    intakeReq = driver.leftTrigger().and(DriverStation::isTeleop).or(Autos.autoIntakeReq);

    antiJamReq = driver.a();

    // TODO add auto climb req
    preClimbReq = driver.x().or(Autos.autoPreClimbReq);
    climbReq = driver.y().or(Autos.autoClimbReq);

    readyTrigger =
        new Trigger(shooter::atFlywheelVelocitySetpoint)
            // .debounce(0.05)
            .and(new Trigger(shooter::atHoodSetpoint).debounce(0.05))
            .and(new Trigger(shooter::atTurretSetpoint).debounce(0.05))
            .and(new Trigger(NewAutoAim::targetInTurretDeadzone).negate());
  }

  private void addTransitions() {
    bindTransition(SuperState.IDLE, SuperState.INTAKE, intakeReq);

    bindTransition(SuperState.INTAKE, SuperState.IDLE, intakeReq.negate());

    // the comp bot has no way of knowing if it's idle or ready
    // which makes me nervous but it's probably fine
    // anyway that's why we don't have a ready state anymore

    bindTransition(
        SuperState.IDLE, SuperState.SPIN_UP_SCORE, shootReq.and(scoreReq).and(flowReq.negate()));

    bindTransition(SuperState.SPIN_UP_SCORE, SuperState.SCORE, readyTrigger);

    // bindTransition(SuperState.SCORE, SuperState.SPIN_UP_SCORE, readyTrigger.negate());

    bindTransition(SuperState.SPIN_UP_SCORE, SuperState.IDLE, shootReq.negate());

    bindTransition(SuperState.SCORE, SuperState.IDLE, shootReq.negate());

    // SCORE_FLOW transitions
    {
      bindTransition(
          SuperState.IDLE,
          SuperState.SPIN_UP_SCORE_FLOW,
          scoreReq.and(flowReq).and(shootReq.or(intakeReq)));

      bindTransition(SuperState.SPIN_UP_SCORE_FLOW, SuperState.SCORE_FLOW, readyTrigger);

      bindTransition(
          SuperState.SCORE_FLOW,
          SuperState.SPIN_UP_SCORE_FLOW,
          new Trigger(NewAutoAim::targetInTurretDeadzone));

      bindTransition(
          SuperState.SPIN_UP_SCORE_FLOW,
          SuperState.IDLE,
          intakeReq.negate().and(shootReq.negate()));

      bindTransition(SuperState.SCORE, SuperState.SCORE_FLOW, flowReq);

      bindTransition(SuperState.SCORE_FLOW, SuperState.SCORE, flowReq.negate());

      bindTransition(
          SuperState.SCORE_FLOW, SuperState.IDLE, intakeReq.negate().and(shootReq.negate()));
    }

    // --------------------------------------------------------------------------

    bindTransition(
        SuperState.IDLE, SuperState.SPIN_UP_FEED, shootReq.and(feedReq).and(flowReq.negate()));

    bindTransition(SuperState.SPIN_UP_FEED, SuperState.FEED, readyTrigger);

    // bindTransition(SuperState.FEED, SuperState.SPIN_UP_FEED, readyTrigger.negate());

    bindTransition(SuperState.SPIN_UP_FEED, SuperState.IDLE, shootReq.negate());

    bindTransition(SuperState.FEED, SuperState.IDLE, shootReq.negate());

    // FEED_FLOW transitions
    {
      bindTransition(
          SuperState.IDLE,
          SuperState.SPIN_UP_FEED_FLOW,
          feedReq.and(flowReq).and(shootReq.or(intakeReq)));

      bindTransition(SuperState.SPIN_UP_FEED_FLOW, SuperState.FEED_FLOW, readyTrigger);

      bindTransition(
          SuperState.FEED_FLOW,
          SuperState.SPIN_UP_FEED_FLOW,
          new Trigger(NewAutoAim::targetInTurretDeadzone));

      bindTransition(
          SuperState.SPIN_UP_FEED_FLOW, SuperState.IDLE, intakeReq.negate().and(shootReq.negate()));

      bindTransition(SuperState.FEED, SuperState.FEED_FLOW, flowReq);

      bindTransition(SuperState.FEED_FLOW, SuperState.FEED, flowReq.negate());

      bindTransition(
          SuperState.FEED_FLOW, SuperState.IDLE, intakeReq.negate().and(shootReq.negate()));
    }

    bindTransition(SuperState.SCORE, SuperState.FEED, feedReq);
    bindTransition(SuperState.FEED, SuperState.SCORE, scoreReq);
    bindTransition(SuperState.SCORE_FLOW, SuperState.FEED_FLOW, feedReq);
    bindTransition(SuperState.FEED_FLOW, SuperState.SCORE_FLOW, scoreReq);

    // Transition from any state to SPIT for anti jamming
    antiJamReq.onTrue(changeStateTo(SuperState.SPIT));

    bindTransition(
        SuperState.SPIT,
        SuperState.IDLE,
        antiJamReq.negate(),
        Commands.runOnce(() -> defense = false));

    defenseReq.onTrue(changeStateTo(SuperState.DEFENSE));

    bindTransition(SuperState.DEFENSE, SuperState.IDLE, defenseReq.negate());

    (preClimbReq.and(climbReq.negate())
        // .and(() -> DriverStation.isTeleop())
        )
        .onTrue(changeStateTo(SuperState.PRE_CLIMB));

    bindTransition(
        SuperState.PRE_CLIMB,
        SuperState.CLIMB,
        climbReq); // TODO maybe add transition out of climb in case we fall
    bindTransition(
        SuperState.PRE_CLIMB,
        SuperState.IDLE,
        preClimbReq.negate().and(climbReq.negate()),
        Commands.runOnce(() -> defense = false));

    bindTransition(
        SuperState.SPIN_UP_SCORE_FLOW,
        SuperState.SPIN_UP_SCORE_PRE_CLIMB,
        new Trigger(() -> DriverStation.isAutonomous()).and(preClimbReq).and(scoreReq));

    bindTransition(
        SuperState.SPIN_UP_SCORE_PRE_CLIMB,
        SuperState.SCORE_PRE_CLIMB,
        new Trigger(() -> DriverStation.isAutonomous())
            .and(preClimbReq)
            .and(scoreReq)
            .and(readyTrigger));

    bindTransition(
        SuperState.SCORE_FLOW,
        SuperState.SCORE_PRE_CLIMB,
        new Trigger(() -> DriverStation.isAutonomous())
            .and(preClimbReq)
            .and(scoreReq)
            .and(readyTrigger));

    bindTransition(
        SuperState.SCORE_PRE_CLIMB,
        SuperState.CLIMB,
        new Trigger(() -> DriverStation.isAutonomous()).and(climbReq));
  }

  private void addCommands() {
    bindCommands(
        SuperState.IDLE,
        intake.restExtended(),
        // intake.restRetracted(),
        indexer.rest(),
        shooter.rest(
            swerve::getPose,
            swerve::getVelocityRobotRelative,
            this::inScoringArea,
            () -> FeedTargets.getFeedTarget(feedTarget).getPose()),
        climber.retract());

    bindCommands(
        SuperState.INTAKE,
        intake.intake(),
        indexer.rest(),
        shooter.rest(
            swerve::getPose,
            swerve::getVelocityRobotRelative,
            this::inScoringArea,
            () -> FeedTargets.getFeedTarget(feedTarget).getPose()),
        climber.retract());

    bindCommands(
        SuperState.SPIN_UP_FEED,
        intake.restExtended(),
        indexer.rest(),
        shooter.feed(
            () ->
                NewAutoAim.getParametersMechA(
                    swerve.getPose(),
                    swerve.getVelocityRobotRelative(),
                    FeedTargets.getFeedTarget(feedTarget).getTranslation(),
                    AutoAim.FEED_SHOT_TREE),
            () -> FeedTargets.getFeedTarget(feedTarget).getPose(),
            swerve::getPose),
        climber.retract());

    bindCommands(
        SuperState.FEED,
        intake.agitate(),
        indexer.kick(),
        shooter.feed(
            () ->
                NewAutoAim.getParametersMechA(
                    swerve.getPose(),
                    swerve.getVelocityRobotRelative(),
                    FeedTargets.getFeedTarget(feedTarget).getTranslation(),
                    AutoAim.FEED_SHOT_TREE),
            () -> FeedTargets.getFeedTarget(feedTarget).getPose(),
            swerve::getPose),
        climber.retract());

    bindCommands(
        SuperState.SPIN_UP_FEED_FLOW,
        intake.intake(),
        indexer.index(),
        shooter.feed(
            () ->
                NewAutoAim.getParametersMechA(
                    swerve.getPose(),
                    swerve.getVelocityRobotRelative(),
                    FeedTargets.getFeedTarget(feedTarget).getTranslation(),
                    AutoAim.FEED_SHOT_TREE),
            () -> FeedTargets.getFeedTarget(feedTarget).getPose(),
            swerve::getPose),
        climber.retract());

    bindCommands(
        SuperState.FEED_FLOW,
        intake.intake(),
        indexer.kick(),
        shooter.feed(
            () ->
                NewAutoAim.getParametersMechA(
                    swerve.getPose(),
                    swerve.getVelocityRobotRelative(),
                    FeedTargets.getFeedTarget(feedTarget).getTranslation(),
                    AutoAim.FEED_SHOT_TREE),
            () -> FeedTargets.getFeedTarget(feedTarget).getPose(),
            swerve::getPose),
        climber.retract());

    bindCommands(
        SuperState.SPIN_UP_SCORE,
        intake.restExtended(),
        indexer.rest(),
        shooter.score(
            () ->
                NewAutoAim.getParametersMechA(
                    swerve.getPose(),
                    swerve.getVelocityRobotRelative(),
                    FieldUtils.getCurrentHubTranslation(),
                    Robot.ROBOT_EDITION == RobotEdition.ALPHA
                        ? AutoAim.ALPHA_HUB_SHOT_TREE
                        : AutoAim.COMP_HUB_SHOT_TREE)),
        climber.retract());

    bindCommands(
        SuperState.SCORE,
        intake.agitate(),
        // intake.restExtended(),
        indexer.kick(),
        shooter.score(
            () ->
                NewAutoAim.getParametersMechA(
                    swerve.getPose(),
                    swerve.getVelocityRobotRelative(),
                    FieldUtils.getCurrentHubTranslation(),
                    Robot.ROBOT_EDITION == RobotEdition.ALPHA
                        ? AutoAim.ALPHA_HUB_SHOT_TREE
                        : AutoAim.COMP_HUB_SHOT_TREE)),
        climber.retract());

    bindCommands(
        SuperState.SPIN_UP_SCORE_FLOW,
        intake.intake(),
        indexer.rest(),
        shooter.score(
            () ->
                NewAutoAim.getParametersMechA(
                    swerve.getPose(),
                    swerve.getVelocityRobotRelative(),
                    FieldUtils.getCurrentHubTranslation(),
                    Robot.ROBOT_EDITION == RobotEdition.ALPHA
                        ? AutoAim.ALPHA_HUB_SHOT_TREE
                        : AutoAim.COMP_HUB_SHOT_TREE)),
        climber.retract());

    bindCommands(
        SuperState.SCORE_FLOW,
        intake.intake(),
        indexer.kick(),
        shooter.score(
            () ->
                NewAutoAim.getParametersMechA(
                    swerve.getPose(),
                    swerve.getVelocityRobotRelative(),
                    FieldUtils.getCurrentHubTranslation(),
                    Robot.ROBOT_EDITION == RobotEdition.ALPHA
                        ? AutoAim.ALPHA_HUB_SHOT_TREE
                        : AutoAim.COMP_HUB_SHOT_TREE)),
        climber.retract());

    bindCommands(
        SuperState.SPIT, intake.outtake(), indexer.spit(), shooter.spit(), climber.retract());
    bindCommands(
        SuperState.PRE_CLIMB,
        intake.restRetracted(),
        indexer.stop(),
        shooter.rest(
            swerve::getPose,
            swerve::getVelocityRobotRelative,
            this::inScoringArea,
            () -> FeedTargets.getFeedTarget(feedTarget).getPose()),
        climber.extend());
    bindCommands(
        SuperState.CLIMB,
        intake.restRetracted(),
        indexer.stop(),
        shooter.rest(
            swerve::getPose,
            swerve::getVelocityRobotRelative,
            this::inScoringArea,
            () -> FeedTargets.getFeedTarget(feedTarget).getPose()),
        climber.retract());
    bindCommands(
        SuperState.POST_CLIMB,
        intake.restRetracted(),
        indexer.stop(),
        shooter.rest(
            swerve::getPose,
            swerve::getVelocityRobotRelative,
            this::inScoringArea,
            () -> FeedTargets.getFeedTarget(feedTarget).getPose()),
        climber.extend());

    bindCommands(
        SuperState.SPIN_UP_SCORE_PRE_CLIMB,
        intake.restRetracted(),
        indexer.rest(),
        shooter.score(
            () ->
                NewAutoAim.getParametersMechA(
                    swerve.getPose(),
                    swerve.getVelocityRobotRelative(),
                    FieldUtils.getCurrentHubTranslation(),
                    Robot.ROBOT_EDITION == RobotEdition.ALPHA
                        ? AutoAim.ALPHA_HUB_SHOT_TREE
                        : AutoAim.COMP_HUB_SHOT_TREE)),
        climber.extend());

    bindCommands(
        SuperState.SCORE_PRE_CLIMB,
        intake.restRetracted(),
        indexer.kick(),
        shooter.score(
            () ->
                NewAutoAim.getParametersMechA(
                    swerve.getPose(),
                    swerve.getVelocityRobotRelative(),
                    FieldUtils.getCurrentHubTranslation(),
                    Robot.ROBOT_EDITION == RobotEdition.ALPHA
                        ? AutoAim.ALPHA_HUB_SHOT_TREE
                        : AutoAim.COMP_HUB_SHOT_TREE)),
        climber.extend());

    bindCommands(
        SuperState.DEFENSE,
        intake.restRetracted(),
        indexer.stop(),
        shooter.stopTurret(),
        climber.retract());
  }

  public void periodic() {
    Logger.recordOutput("Superstructure/Superstructure State", state);
    Logger.recordOutput("Superstructure/State Timer", stateTimer.get());

    // this really should be in robot.java but i cooked myself with the robot selecting thing
    Logger.recordOutput(
        "shooter sotm viz",
        new Pose3d(swerve.getPose())
            .transformBy(
                new Transform3d(
                    new Translation3d(0, 0, 0.5),
                    new Rotation3d(
                        0, ((Math.PI / 2) - shooter.getHoodSetpoint().getRadians()) * -1, 0))));
  }

  /**
   * @param start first state
   * @param end second state
   * @param trigger trigger to make it go from the first state to the second (assuming it's already
   *     in the first state)
   */
  private void bindTransition(SuperState start, SuperState end, Trigger trigger) {
    // when 1) the robot is in the start state and 2) the trigger is true, the robot changes state
    // to the end state
    trigger.and(start.getTrigger()).onTrue(changeStateTo(end));
  }

  /**
   * @param start first state
   * @param end second state
   * @param trigger trigger to make it go from the first state to the second (assuming it's already
   *     in the first state)
   * @param cmd some command to run while making the transition
   */
  private void bindTransition(SuperState start, SuperState end, Trigger trigger, Command cmd) {
    // when 1) the robot is in the start state and 2) the trigger is true, the robot changes state
    // to the end state IN PARALLEL to running the command that got passed in
    trigger.and(start.getTrigger()).onTrue(Commands.parallel(changeStateTo(end), cmd));
  }

  /**
   * Runs the passed in command(s) in parallel when the superstructure is in the passed in state
   *
   * @param state
   * @param commands
   */
  private void bindCommands(SuperState state, Command... commands) {
    state.getTrigger().whileTrue(Commands.parallel(commands));
  }

  // public boolean atExtension(SuperState state) {
  // }

  // public boolean atExtension() {
  //   return atExtension(state);
  // }

  private Command changeStateTo(SuperState nextState) {
    return Commands.runOnce(
            () -> {
              System.out.println("Changing state from " + state + " to " + nextState);
              stateTimer.reset();
              this.prevState = state;
              state = nextState;
              setSubstates();
            })
        .ignoringDisable(true)
        .withName("State Change Command");
  }

  private void setSubstates() {}

  // public Command transitionAfterZeroing() {
  //  }

  /**
   * <b>Only for setting initial state at the beginning of auto</b>
   *
   * @param state the state to set to
   */
  public void resetStateForAuto(SuperState nextState) {
    System.out.println("Resetting state from " + state + " to " + nextState + " for auto.");
    stateTimer.reset();
    this.prevState = state;
    state = nextState;
    setSubstates();
  }

  public static SuperState getState() {
    return state;
  }

  public boolean stateIsIdle() {
    return getState() == SuperState.IDLE;
  }

  private Alliance getStartingAlliance() {
    String gameData = DriverStation.getGameSpecificMessage();
    // gives first inactive alliance
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          return Alliance.Red;
        case 'R':
          return Alliance.Blue;
        default:
          return Alliance.Blue;
      }
    } else {
      // not sure
      return Alliance.Blue;
    }
  }

  private String getCurrentShift() {
    if (DriverStation.isDisabled()) return "Disabled";
    if (130.00 < timeLeftinMatch() && timeLeftinMatch() <= 140.00) {
      return "Transition";
    } else if (105.00 < timeLeftinMatch() && timeLeftinMatch() <= 130.00) {
      return "Shift 1";
    } else if (80.00 < timeLeftinMatch() && timeLeftinMatch() <= 105.00) {
      return "Shift 2";
    } else if ((55.00 < timeLeftinMatch() && timeLeftinMatch() <= 80.00)) {
      return "Shift 3";
    } else if ((30.00 < timeLeftinMatch() && timeLeftinMatch() <= 55.00)) {
      return "Shift 4";
    } else {
      return "End Game";
    }
  }

  private double getTimeLeftInShift() {
    if (DriverStation.isDisabled()) return 0;
    double offset =
        switch (getCurrentShift()) {
          case "Transition" -> 130.00;
          case "Shift 1" -> 105.00;
          case "Shift 2" -> 80.00;
          case "Shift 3" -> 55.00;
          case "Shift 4" -> 30.00;
          default -> 0.00;
        };
    return timeLeftinMatch() - offset;
  }

  @AutoLogOutput(key = "Is our shift?")
  public boolean isOurShift() {
    if (DriverStation.isDisabled()) return false;
    // only cant score when its the others turn, otherwise everyone can
    if (getStartingAlliance() == DriverStation.getAlliance().orElse(Alliance.Blue)) {
      return !(getCurrentShift() == "Shift 2" || getCurrentShift() == "Shift 4");
    } else {
      return !(getCurrentShift() == "Shift 1" || getCurrentShift() == "Shift 3");
    }
  }

  public boolean tenSecsLeftInOffShift() {
    if (!isOurShift() && (10.0 <= getTimeLeftInShift() && getTimeLeftInShift() <= 11.0)) {
      return true;
    } else {
      return false;
    }
  }

  @AutoLogOutput(key = "10s Left (in off shift)")
  public boolean lessThanTenSecsLeftInOffShift() {
    if (!isOurShift() && (10.0 <= getTimeLeftInShift())) {
      return true;
    } else {
      return false;
    }
  }

  public boolean inScoringArea() {
    // return true;
    if (swerve == null) return false;
    return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            && (swerve.getPose().getX() <= 4.6914191246032715))
        || (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            && (swerve.getPose().getX() >= 11.889562606811523));
  }

  public boolean canScore() {
    return
    // (isOurShift() || !DriverStation.isFMSAttached())
    //     &&
    (inScoringArea() || poseOverride)
        && (!swerve.isNearTrench() || poseOverride
        // || fixedShotTarget != FixedShotTarget.NONE
        );
  }

  public boolean canShoot() {
    return !swerve.isNearTrenchForHood();
  }

  public static ShotTarget getShotTarget() {
    return shotTarget;
  }

  public static FeedTarget getFeedTarget() {
    return feedTarget;
  }

  // public static FixedShotTarget getFixedShotTarget() {
  //   return fixedShotTarget;
  // }

  public static boolean getPoseOverride() {
    return poseOverride;
  }
}
