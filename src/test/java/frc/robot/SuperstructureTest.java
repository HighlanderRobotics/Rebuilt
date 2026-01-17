package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure.SuperState;
import frc.robot.components.rollers.RollerIOReal;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SuperstructureTest {
  Superstructure superstructure;
  IntakeSubsystem intake;
  HoodSubsystem shooter;
  IndexerSubsystem routing;
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
    isEmpty = false;

    intake = new IntakeSubsystem(new RollerIOReal(10, IntakeSubsystem.getIntakeIOConfig()));
    shooter = new HoodSubsystem(new HoodIOSim(new CANBus()));
    routing =
        new IndexerSubsystem(
            new CANBus(), new RollerIOReal(11, IndexerSubsystem.getIndexerConfigs()));
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
            new Trigger(() -> flowReq),
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
        Superstructure.getState(), SuperState.IDLE); // Verify that superstructure starts in IDLE

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(
        Superstructure.getState(),
        SuperState.IDLE); // Verify that the superstructure hasn't transitioned yet

    intakeReq = true; // This should trigger the state transition from IDLE to INTAKE

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(
        Superstructure.getState(),
        SuperState.INTAKE); // Verify that the superstructure has properly transitioned

    // TODO: THIS DOESN'T WORK BC THE AREN'T THE SAME COMMAND IN MEMORY. FIGURE OUT HOW TO FIX
    // assertEquals(intake.getCurrentCommand(), intake.intake()); // Verify that the intake is
    // intaking
  }

  @Test
  void shouldFail() {
    assertEquals(1, 2);
  }
}
