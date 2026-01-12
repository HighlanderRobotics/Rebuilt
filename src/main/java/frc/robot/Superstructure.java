// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    IDLE(),
    INTAKE(),
    READY(),
    FEED(),
    FEED_FLOW(),
    SCORE(),
    SCORE_FLOW(),
    SPIT();
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

  private final SwerveSubsystem swerve;
  private final CommandXboxControllerSubsystem driver;
  private final CommandXboxControllerSubsystem operator;

  // Declare triggers
  @AutoLogOutput(key = "Superstructure/Score Request")
  private Trigger scoreReq;

  @AutoLogOutput(key = "Superstructure/Intake Request")
  private Trigger intakeReq;

  @AutoLogOutput(key = "Superstructure/Feed Request")
  private Trigger feedReq;

  @AutoLogOutput(key = "Superstructre/Continuous Request")
  private Trigger continuousReq;

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
      CommandXboxControllerSubsystem driver,
      CommandXboxControllerSubsystem operator) {
    this.swerve = swerve;
    this.driver = driver;
    this.operator = operator;

    addTriggers();
    addTransitions();

    stateTimer.start();
  }

  private void addTriggers() {
    scoreReq =
        driver
            .rightTrigger()
            .negate()
            .and(DriverStation::isTeleop)
            .or(Autos.autoScoreReq); // Maybe should include if its our turn?
  }

  private void addTransitions() {
    bindTransition(SuperState.IDLE, SuperState.INTAKE, intakeReq);

    bindTransition(SuperState.INTAKE, SuperState.READY, intakeReq.negate().or(isFull));

    bindTransition(SuperState.READY, SuperState.INTAKE, intakeReq.and(isFull.negate()));

    bindTransition(SuperState.READY, SuperState.FEED, feedReq.and(continuousReq.negate()));

    bindTransition(SuperState.FEED, SuperState.READY, feedReq.negate().and(isEmpty.negate()));

    bindTransition(
        SuperState.FEED,
        SuperState.IDLE,
        isEmpty.and(
            feedReq.negate())); // This is the condition in the graph. Should it just transition
    // automatically when empty?

    bindTransition(SuperState.READY, SuperState.SCORE, scoreReq.and(continuousReq.negate()));

    bindTransition(SuperState.SCORE, SuperState.READY, scoreReq.negate().and(isEmpty.negate()));

    bindTransition(SuperState.SCORE, SuperState.IDLE, scoreReq.negate().and(isEmpty));

    // FEED_FLOW transitions
    {
      bindTransition(SuperState.IDLE, SuperState.FEED_FLOW, feedReq.and(continuousReq));

      bindTransition(SuperState.INTAKE, SuperState.FEED_FLOW, feedReq.and(continuousReq));

      bindTransition(SuperState.FEED, SuperState.FEED_FLOW, feedReq.and(continuousReq));
      // Graph has no transition from READY to FEED_FLOW. I think the transition should be added
      // though.

      bindTransition(SuperState.FEED_FLOW, SuperState.FEED, feedReq.and(continuousReq.negate()));

      bindTransition(
          SuperState.FEED_FLOW, SuperState.READY, feedReq.negate().and(isEmpty.negate()));

      bindTransition(SuperState.FEED_FLOW, SuperState.IDLE, feedReq.negate().and(isEmpty));
    }

    // SCORE_FLOW transitions
    {
      bindTransition(SuperState.IDLE, SuperState.SCORE_FLOW, scoreReq.and(continuousReq));

      bindTransition(SuperState.SCORE, SuperState.SCORE_FLOW, scoreReq.and(continuousReq));

      bindTransition(SuperState.INTAKE, SuperState.SCORE_FLOW, scoreReq.and(continuousReq));
      // Graph has no transition from READY to SCORE_FLOW. I think it should be added

      bindTransition(SuperState.SCORE_FLOW, SuperState.IDLE, scoreReq.negate().and(isEmpty));

      bindTransition(
          SuperState.SCORE_FLOW, SuperState.READY, scoreReq.negate().and(isEmpty.negate()));

      bindTransition(SuperState.SCORE_FLOW, SuperState.SCORE, scoreReq.and(continuousReq.negate()));
    }

    // Transition from any state to SPIT for anti jamming
    antiJamReq.onTrue(changeStateTo(SuperState.SPIT));

    bindTransition(SuperState.SPIT, SuperState.IDLE, antiJamReq.negate());
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
}
