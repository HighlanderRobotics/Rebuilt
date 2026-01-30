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
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.FeedTargets;
import frc.robot.utils.autoaim.AutoAim;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  /** We should have a state for every single action the robot will perform. */
  public enum SuperState {
    IDLE,
    INTAKE,
    READY,
    SPIN_UP_FEED,
    FEED,
    FEED_FLOW,
    SPIN_UP_SCORE,
    SCORE,
    SCORE_FLOW,
    SPIT;
    public final Trigger trigger;

    private SuperState() {
      trigger = new Trigger(() -> state == this);
    }

    public Trigger getTrigger() {
      return trigger;
    }
  }

  @AutoLogOutput(key = "Superstructure/State")
  private static SuperState state = SuperState.IDLE;

  @AutoLogOutput(key = "Scoring/Scoring Active")
  public boolean isScoringActive =
      isOurShift(); // assuming we want the dashboard to show if the time allows us to score not if

  public boolean practice = true;

  // its litterly possible

  private SuperState prevState = SuperState.IDLE;

  private Timer stateTimer = new Timer();

  private final SwerveSubsystem swerve;
  private final Indexer indexer;
  private final Intake intake;
  private final Shooter shooter;
  private final CommandXboxControllerSubsystem driver;
  private final CommandXboxControllerSubsystem operator;

  // Declare triggers
  @AutoLogOutput(key = "Superstructure/Score Request")
  private Trigger scoreReq;

  @AutoLogOutput(key = "Superstructure/Intake Request")
  private Trigger intakeReq;

  @AutoLogOutput(key = "Superstructure/Feed Request")
  private Trigger feedReq;

  // @AutoLogOutput(key = "Superstructure/Flowstate Request")
  // private Trigger flowReq;

  @AutoLogOutput(key = "Superstructure/Anti Jam Req")
  private Trigger antiJamReq;

  @AutoLogOutput(key = "Superstructure/Is Full")
  private Trigger isFull;

  @AutoLogOutput(key = "Superstructure/Is Empty")
  private Trigger isEmpty;

  private boolean shouldFeed = false;

  // @AutoLogOutput(key = "Superstructure/At Extension?")
  // public Trigger atExtensionTrigger = new Trigger(this::atExtension).or(Robot::isSimulation);

  /** Creates a new Superstructure. */
  public Superstructure(
      SwerveSubsystem swerve,
      Indexer indexer,
      Intake intake,
      Shooter shooter,
      CommandXboxControllerSubsystem driver,
      CommandXboxControllerSubsystem operator) {
    this.swerve = swerve;
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
    this.driver = driver;
    this.operator = operator;

    addTriggers();
    addTransitions();
    addCommands();

    stateTimer.start();
  }

  private void addTriggers() {
    // Toggles for feeding
    operator.leftBumper().onTrue(Commands.runOnce(() -> shouldFeed = true));
    operator.rightBumper().onTrue(Commands.runOnce(() -> shouldFeed = false));

    scoreReq =
        driver
            .rightTrigger()
            .and(DriverStation::isTeleop)
            .and(() -> canScore())
            .or(Autos.autoScoreReq); // Maybe should include if its our turn?

    intakeReq = driver.leftTrigger().and(DriverStation::isTeleop).or(Autos.autoIntakeReq);

    feedReq = driver.rightBumper().and(DriverStation::isTeleop).or(Autos.autoFeedReq);

    // flowReq = driver.leftTrigger().and(driver.rightTrigger());

    antiJamReq = driver.a().or(operator.a());

    isFull = new Trigger(indexer::isFull).debounce(0.5); // TODO tune

    isEmpty = new Trigger(indexer::isEmpty);
  }

  private void addTransitions() {
    bindTransition(SuperState.IDLE, SuperState.INTAKE, intakeReq.and(scoreReq.negate()));

    bindTransition(SuperState.INTAKE, SuperState.IDLE, intakeReq.negate().and(isEmpty));

    bindTransition(
        SuperState.INTAKE,
        SuperState.READY,
        (intakeReq.negate().and(scoreReq.negate()).and(isEmpty.negate())));
    // .or(isFull));

    // bindTransition(SuperState.INTAKE, SuperState.SPIN_UP_FEED, feedReq);

    bindTransition(SuperState.READY, SuperState.INTAKE, intakeReq);
    // .and(isFull.negate()));

    bindTransition(SuperState.READY, SuperState.SPIN_UP_SCORE, scoreReq);

    bindTransition(
        SuperState.SPIN_UP_SCORE,
        SuperState.SCORE,
        new Trigger(shooter::atFlywheelVelocitySetpoint)
            .debounce(0.5)
            .and(new Trigger(shooter::atHoodSetpoint).debounce(0.5))
            .and(() -> stateTimer.hasElapsed(0.5)));

    // bindTransition(
    //     SuperState.SPIN_UP_FEED,
    //     SuperState.FEED,
    //     new Trigger(shooter::atFlywheelVelocitySetpoint)
    //         .and(() -> stateTimer.hasElapsed(0.2))
    //         .and(shooter::atHoodSetpoint));

    // bindTransition(SuperState.FEED, SuperState.IDLE, isEmpty);

    bindTransition(SuperState.SCORE, SuperState.IDLE, isEmpty.debounce(0.5).and(scoreReq.negate()));

    // FEED_FLOW transitions
    // {
    //   bindTransition(SuperState.FEED, SuperState.FEED_FLOW, intakeReq.and(feedReq));

    //   bindTransition(SuperState.FEED_FLOW, SuperState.FEED, intakeReq.negate().and(feedReq));

    //   bindTransition(
    //       SuperState.FEED_FLOW, SuperState.READY, flowReq.negate().and(isEmpty.negate()));

    //   // No so sure about the end condition here.
    //   bindTransition(SuperState.FEED_FLOW, SuperState.IDLE, flowReq.negate().and(isEmpty));
    // }

    // SCORE_FLOW transitions
    {
      bindTransition(SuperState.SCORE, SuperState.SCORE_FLOW, scoreReq.and(intakeReq));

      bindTransition(SuperState.SCORE_FLOW, SuperState.SCORE, intakeReq.negate().and(scoreReq));

      bindTransition(
          SuperState.SCORE_FLOW,
          SuperState.READY,
          intakeReq.negate().and(scoreReq.negate()).and(isEmpty.negate()));

      // No so sure about the end condition here.
      bindTransition(
          SuperState.SCORE_FLOW,
          SuperState.IDLE,
          intakeReq.negate().and(scoreReq.negate()).and(isEmpty));
    }

    // Transition from any state to SPIT for anti jamming
    antiJamReq.onTrue(changeStateTo(SuperState.SPIT));

    bindTransition(SuperState.SPIT, SuperState.IDLE, antiJamReq.negate());
  }

  private void addCommands() {
    bindCommands(
        SuperState.IDLE,
        intake.rest(),
        indexer.rest(),
        shooter.rest()); // Maybe the indexer should be indexing?

    bindCommands(SuperState.INTAKE, intake.intake(), indexer.index(), shooter.rest());

    bindCommands(
        SuperState.READY,
        intake.rest(),
        indexer.rest(),
        shooter.rest()); // Maybe index at slower speed?

    bindCommands(
        SuperState.SPIN_UP_SCORE,
        intake.rest(),
        indexer.rest(), /*shooter.shoot(swerve::getPose)*/
        shooter.shoot(
            () ->
                AutoAim.getCompensatedSOTMShotData(
                    swerve.getPose(),
                    FieldUtils.getCurrentHubTranslation(),
                    swerve.getVelocityFieldRelative())));
    // shooter.testShoot());

    bindCommands(
        SuperState.SPIN_UP_FEED,
        intake.rest(),
        indexer.rest(),
        shooter.feed(
            swerve::getPose, () -> FeedTargets.BLUE_BACK_RIGHT.getPose())); // TODO: SELECTION LOGIC

    bindCommands(
        SuperState.SCORE,
        intake.rest(),
        indexer.kick(), /*shooter.shoot(swerve::getPose)*/
        shooter.shoot(
            () ->
                AutoAim.getCompensatedSOTMShotData(
                    swerve.getPose(),
                    FieldUtils.getCurrentHubTranslation(),
                    swerve.getVelocityFieldRelative())));
    // shooter.testShoot());

    bindCommands(SuperState.SCORE_FLOW, intake.intake(), indexer.kick(), shooter.testShoot());

    bindCommands(
        SuperState.FEED,
        intake.rest(),
        indexer.index(),
        shooter.feed(
            swerve::getPose,
            () -> FeedTargets.BLUE_BACK_RIGHT.getPose())); // TODO: ADD SOME SELECTION LOGIC

    bindCommands(
        SuperState.FEED_FLOW,
        intake.intake(),
        indexer.index(),
        shooter.feed(swerve::getPose, () -> FeedTargets.BLUE_BACK_RIGHT.getPose()));

    bindCommands(SuperState.SPIT, intake.outtake(), indexer.spit(), shooter.spit());
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

  private int getCurrentShift() {
    double timeLeftinMatch = Timer.getMatchTime();
    // may be a nicer way to do this
    if (105.00 <= timeLeftinMatch && timeLeftinMatch <= 130.00) {
      return 1;
    } else if (80.00 <= timeLeftinMatch && timeLeftinMatch <= 105.00) {
      return 2;
    } else if ((55.00 <= timeLeftinMatch && timeLeftinMatch <= 80.00)) {
      return 3;
    } else if ((30.00 <= timeLeftinMatch && timeLeftinMatch <= 55.00)) {
      return 4;
    } else {
      return 0;
    }
  }

  public boolean isOurShift() {
    // only cant score when its the others turn, otherwise everyone can
    if (getStartingAlliance() == DriverStation.getAlliance().orElse(Alliance.Blue)) {
      return !(getCurrentShift() == 2 || getCurrentShift() == 4);
    } else {
      return !(getCurrentShift() == 1 || getCurrentShift() == 3);
    }
  }

  public boolean inScoringArea() {
    return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            && (swerve.getPose().getX() <= 4.6914191246032715)
        || DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            && (swerve.getPose().getX() >= 11.889562606811523));
  }

  public boolean canScore() {
    return isOurShift() && inScoringArea() && practice;
  }
}
