// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.CommandXboxControllerSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  /**
   * We should have a state for every single "pose" the robot will hit. See this document for
   * screenshots of the robot in each state. There are also named positions in cad for each state.
   */
  public enum SuperState {
    IDLE,
    INTAKE,
    READY,
    FEED,
    FEED_FLOW,
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

  private SuperState prevState = SuperState.IDLE;

  private Timer stateTimer = new Timer();
  public Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

  private final SwerveSubsystem swerve;
  private final RoutingSubsystem routing;
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final CommandXboxControllerSubsystem driver;
  private final CommandXboxControllerSubsystem operator;

  // Declare triggers
  @AutoLogOutput(key = "Superstructure/Score Request")
  private Trigger scoreReq;

  @AutoLogOutput(key = "Superstructure/Intake Request")
  private Trigger intakeReq;

  @AutoLogOutput(key = "Superstructure/Feed Request")
  private Trigger feedReq;

  @AutoLogOutput(key = "Superstructre/Flowstate Request")
  private Trigger flowReq;

  @AutoLogOutput(key = "Superstructre/Anti Jam Req")
  private Trigger antiJamReq;

  @AutoLogOutput(key = "Superstructure/Is Full")
  private Trigger isFull;

  @AutoLogOutput(key = "Superstructure/Is Empty")
  private Trigger isEmpty;

  // @AutoLogOutput(key = "Superstructure/At Extension?")
  // public Trigger atExtensionTrigger = new Trigger(this::atExtension).or(Robot::isSimulation);

  /** Creates a new Superstructure. */
  public Superstructure(
      SwerveSubsystem swerve,
      RoutingSubsystem routing,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      CommandXboxControllerSubsystem driver,
      CommandXboxControllerSubsystem operator) {
    this.swerve = swerve;
    this.routing = routing;
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
    // TODO: THESE BINDINGS WILL LIKELY CHANGE. SHOULD HAVE A FULL MEETING TO DISCUSS
    scoreReq =
        driver
            .rightTrigger()
            .and(DriverStation::isTeleop)
            .and(() -> canScore())
            .or(Autos.autoScoreReq); // Maybe should include if its our turn?

    intakeReq = driver.leftTrigger().and(DriverStation::isTeleop).or(Autos.autoIntakeReq);

    // or should it be like the same button/general req for feeding vs scoring and choose based on
    // if it can score or operator can override??
    feedReq =
        driver
            .rightBumper()
            .and(DriverStation::isTeleop)
            .and(() -> !inScoringArea())
            .or(Autos.autoFeedReq);

    flowReq = operator.rightTrigger();

    antiJamReq = driver.a().or(operator.a());

    isFull = new Trigger(routing::isFull);

    isEmpty = new Trigger(routing::isEmpty);
  }

  private void addTransitions() {
    bindTransition(SuperState.IDLE, SuperState.INTAKE, intakeReq);

    bindTransition(SuperState.INTAKE, SuperState.IDLE, intakeReq.negate().and(isEmpty));

    bindTransition(
        SuperState.INTAKE, SuperState.READY, (intakeReq.negate().and(isEmpty.negate())).or(isFull));

    bindTransition(SuperState.READY, SuperState.INTAKE, intakeReq.and(isFull.negate()));

    bindTransition(SuperState.READY, SuperState.FEED, feedReq);

    bindTransition(SuperState.FEED, SuperState.IDLE, isEmpty);

    bindTransition(SuperState.READY, SuperState.SCORE, scoreReq);

    bindTransition(SuperState.SCORE, SuperState.IDLE, isEmpty);

    // FEED_FLOW transitions
    {
      bindTransition(SuperState.FEED, SuperState.FEED_FLOW, flowReq);

      // No so sure about the end condition here.
      bindTransition(SuperState.FEED_FLOW, SuperState.IDLE, flowReq.negate());

      // Maybe should be a transition from idle to flow as well? In case robot doesn't already have
      // a fuel
    }

    // SCORE_FLOW transitions
    {
      bindTransition(SuperState.SCORE, SuperState.SCORE_FLOW, flowReq);

      bindTransition(SuperState.SCORE_FLOW, SuperState.IDLE, flowReq.negate());
      // Maybe should be a transition from idle to flow as well? In case robot doesn't already have
      // a fuel
    }

    // Transition from any state to SPIT for anti jamming
    antiJamReq.onTrue(changeStateTo(SuperState.SPIT));

    bindTransition(SuperState.SPIT, SuperState.IDLE, antiJamReq.negate());
  }

  private void addCommands() {
    bindCommands(
        SuperState.IDLE,
        intake.rest(),
        routing.rest(),
        shooter.rest()); // Maybe the routing should be indexing?

    bindCommands(SuperState.INTAKE, intake.intake(), routing.index(), shooter.rest());

    bindCommands(
        SuperState.READY,
        intake.rest(),
        routing.index(),
        shooter.rest()); // Maybe index at slower speed?

    bindCommands(SuperState.SCORE, intake.rest(), routing.index(), shooter.shoot());

    bindCommands(SuperState.SCORE_FLOW, intake.intake(), routing.index(), shooter.shoot());

    bindCommands(SuperState.FEED, intake.rest(), routing.index(), shooter.feed());

    bindCommands(SuperState.FEED_FLOW, intake.intake(), routing.index(), shooter.feed());

    bindCommands(SuperState.SPIT, intake.spit(), routing.reverseIndex(), shooter.shoot());
  }

  public void periodic() {
    Logger.recordOutput("Superstructure/Superstructure State", state);
    Logger.recordOutput("Superstructure/State Timer", stateTimer.get());
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

  public boolean isOurShift() {
    String gameData = DriverStation.getGameSpecificMessage();
    boolean blueStarts = false;
    boolean redStarts = false;
    double timeLeftinMatch = Timer.getMatchTime();
    boolean isShiftOne;

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          blueStarts = true;
          break;
        case 'R':
          redStarts = true;
          break;
        default:
          break;
      }
    } else {
      // not sure
      return false;
    }

    // TODO shorten and fix this ugly ahh code
    isShiftOne = (blueStarts && alliance == Alliance.Blue || redStarts && alliance == Alliance.Red);

    // Obviously fix naming :( and thers prob a prettier way to do ts
    boolean shiftOneandthree =
        ((105.00 <= Timer.getMatchTime() && Timer.getMatchTime() <= 130.00)
            || (55.00 <= Timer.getMatchTime() && Timer.getMatchTime() <= 80.00));

    boolean shifttwoandfour =
        ((80.00 <= Timer.getMatchTime() && Timer.getMatchTime() <= 105.00)
            || (30.00 <= Timer.getMatchTime() && Timer.getMatchTime() <= 55.00));

    // make this like a loop or list for efficency
    if (isShiftOne) {
      return !shifttwoandfour;
    } else {
      return !shiftOneandthree;
    }
  }

  public boolean inScoringArea() {
    return (alliance == Alliance.Blue && (swerve.getPose().getX() <= 4.6914191246032715)
        || alliance == Alliance.Red && (swerve.getPose().getX() >= 11.889562606811523));
  }

  public boolean canScore() {
    return isOurShift() && inScoringArea();
  }
}
