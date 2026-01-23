package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure.SuperState;
import frc.robot.components.rollers.RollerIO;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.LindexerSubsystem;
import frc.robot.subsystems.intake.FintakeSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.LintakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.HoodIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SuperstructureTest {
  Superstructure superstructure;
  Intake intake;
  Shooter shooter;
  Indexer routing;
  SwerveSubsystem swerve;

  boolean scoreReq;
  boolean intakeReq;
  boolean feedReq;
  boolean flowReq;
  boolean antiJamReq;
  boolean isFull;
  boolean isEmpty;

  @BeforeEach // Runs before each test
  void setup() {
    // TODO: SETUP
    // Reset command scheduler
    Field instance;
    try {
      instance = CommandScheduler.class.getDeclaredField("instance");
      instance.setAccessible(true);
      instance.set(null, null);
    } catch (Exception e) {
      e.printStackTrace();
    }

    assert HAL.initialize(500, 0); // Initialize HAL, crash if failed

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    scoreReq = false;
    intakeReq = false;
    feedReq = false;
    flowReq = false;
    antiJamReq = false;
    isFull = false;
    isEmpty = true;

    intake = new FintakeSubsystem(new RollerIO(10, FintakeSubsystem.getIntakeConfig(), new CANBus()));
    shooter =
        new ShooterSubsystem(
            new HoodIO(HoodIO.getHoodConfiguration(), new CANBus()),
            new FlywheelIO(FlywheelIO.getFlywheelConfiguration(), new CANBus()));
    routing =
        new LindexerSubsystem(
            new CANBus(),
            new RollerIO(11, LindexerSubsystem.getIndexerConfigs(), new CANBus()),
            new RollerIO(12, LindexerSubsystem.getKickerConfigs(), new CANBus()));
    swerve = new SwerveSubsystem(new CANBus());

    superstructure =
        new Superstructure(
            swerve,
            routing,
            intake,
            shooter,
            new Trigger(() -> scoreReq),
            new Trigger(() -> intakeReq),
            new Trigger(() -> feedReq),
            new Trigger(() -> antiJamReq),
            new Trigger(() -> isFull),
            new Trigger(() -> isEmpty));
  }

  @AfterEach // Runs after each test
  void tearDown() {
    // TODO: TEARDOWN
    try {
      superstructure.close();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Test
  void idleToIntake() {
    assertEquals(
        SuperState.IDLE, Superstructure.getState()); // Verify that superstructure starts in IDLE

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(
        SuperState.IDLE,
        Superstructure.getState()); // Verify that the superstructure hasn't transitioned yet

    intakeReq = true; // This should trigger the state transition from IDLE to INTAKE

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(
        SuperState.INTAKE,
        Superstructure.getState()); // Verify that the superstructure has properly transitioned

    // TODO: THIS DOESN'T WORK BC THE AREN'T THE SAME COMMAND IN MEMORY. FIGURE OUT HOW TO FIX
    // assertEquals(intake.getCurrentCommand(), intake.intake()); // Verify that the intake is
    // intaking
  }

  @Test
  void intakeToReadyNotFull() {
    idleToIntake(); // First, we need to get into intake

    isEmpty = false; // We're no longer empty

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(
        SuperState.INTAKE,
        Superstructure.getState()); // Should still be intaking bc the request is not off

    intakeReq = false;

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(
        SuperState.READY,
        Superstructure
            .getState()); // Should be in READY because we're not empty and intakeReq is false
  }

  @Test
  void intakeToReadyFull() {
    idleToIntake(); // Enter Intake

    isEmpty = false; // We have at least 1 ball

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(
        SuperState.INTAKE,
        Superstructure
            .getState()); // Should still be intaking because we're not full and the request is not
    // off

    isFull = true; // Full

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(SuperState.READY, Superstructure.getState()); // Should be ready because we're full
    assertEquals(true, intakeReq); // Even though we're still requesting to intake
  }

  @Test
  void readyToSpinUpScore() {
    intakeToReadyNotFull(); // Get into ready

    scoreReq = true; // I.e. press button to start scoring

    //
    CommandScheduler.getInstance().run();

    // Same note as readyToSpinUpFeed
    assertEquals(SuperState.SPIN_UP_SCORE, Superstructure.getState());
  }

  @Test
  void scoreToIdle() {
    // readyToScore(); // Start scoring

    assertEquals(SuperState.SCORE, Superstructure.getState()); // Ensure we're still scoring
    assertEquals(true, scoreReq);

    scoreReq = false;

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(
        SuperState.SCORE,
        Superstructure.getState()); // Should still score because we only transition when empty

    isEmpty = true; // We've shot our whole hopper

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(SuperState.IDLE, Superstructure.getState());
  }

  @Test
  void readyToSpinUpFeed() {
    intakeToReadyNotFull(); // Get into ready

    feedReq = true; // I.e. press button to start scoring

    // One cycle to change states
    CommandScheduler.getInstance().run();
    // I believe this test is failing bc when the check runs, the flywheel actual and setpoint
    // velocity are both zero. Fixed in bringup by adding a debounce. When that gets merged, will
    // work on maiking this pass
    assertEquals(SuperState.SPIN_UP_FEED, Superstructure.getState());
  }

  @Test
  void feedToIdle() {
    // readyToFeed(); // Start feeding

    assertEquals(SuperState.FEED, Superstructure.getState()); // Ensure we're still scoring
    assertEquals(true, feedReq);

    feedReq = false;

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(
        SuperState.FEED,
        Superstructure.getState()); // Should still score because we only transition when empty

    isEmpty = true; // We've shot our whole hopper

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(SuperState.IDLE, Superstructure.getState());
  }

  @Test
  void feedToFeedFlow() {
    // readyToFeed(); // Get into feed

    assertEquals(SuperState.FEED, Superstructure.getState()); // Ensure we're still feeding
    assertEquals(true, feedReq); // Make sure we're still requesting to feed

    flowReq = true; // Request to flow

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(SuperState.FEED_FLOW, Superstructure.getState()); // Should be in FEED_FLOW now
  }

  @Test
  void scoreToScoreFlow() {
    // readyToScore(); // Get into score

    assertEquals(SuperState.SCORE, Superstructure.getState()); // Ensure we're still feeding
    assertEquals(true, scoreReq); // Make sure we're still requesting to feed

    flowReq = true; // Request to flow

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(SuperState.SCORE_FLOW, Superstructure.getState()); // Should be in FEED_FLOW now
  }

  @Test
  void idleToFeedFlow() {
    assertEquals(SuperState.IDLE, Superstructure.getState()); // Ensure we start in IDLE
    assertEquals(feedReq, false);

    flowReq = true; // We want to flow

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(
        SuperState.IDLE,
        Superstructure.getState()); // Shouldn't transition yet bc feedReq is still false

    feedReq = true; // We want to feed

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(SuperState.FEED_FLOW, Superstructure.getState()); // Should be in FEED_FLOW now
  }

  @Test
  void idleToScoreFlow() {
    assertEquals(SuperState.IDLE, Superstructure.getState()); // Ensure we start in IDLE
    assertEquals(scoreReq, false);

    flowReq = true; // We want to flow

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(
        SuperState.IDLE,
        Superstructure.getState()); // Shouldn't transition yet bc scoreReq is still false

    scoreReq = true; // We want to score

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(SuperState.SCORE_FLOW, Superstructure.getState()); // Should be in SCORE_FLOW now
  }

  @Test
  void feedFlowToIdle() {
    idleToFeedFlow(); // Get into feed flow
    assertEquals(SuperState.FEED_FLOW, Superstructure.getState());
    assertEquals(true, flowReq);
    assertEquals(true, feedReq);

    flowReq = false;

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(SuperState.IDLE, Superstructure.getState());
  }

  @Test
  void scoreFlowToIdle() {
    idleToScoreFlow(); // Get into feed flow
    assertEquals(SuperState.SCORE_FLOW, Superstructure.getState());
    assertEquals(true, flowReq);
    assertEquals(true, scoreReq);

    flowReq = false;

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(SuperState.IDLE, Superstructure.getState());
  }
}
