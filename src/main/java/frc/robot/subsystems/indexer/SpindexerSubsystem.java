package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Spindexer = Spinning Indexer. !! COMP !! */
public class SpindexerSubsystem extends SubsystemBase implements Indexer {

  public static final double SPINNER_GEAR_RATIO = 67.0 / 12.0;
  public static final double KICKER_GEAR_RATIO = 24.0 / 18.0;

  private RollerIO spinnerIO;

  RollerIOInputsAutoLogged spinnerInputs = new RollerIOInputsAutoLogged();

  RollerIO kickerIO;
  RollerIOInputsAutoLogged kickerInputs = new RollerIOInputsAutoLogged();

  private SysIdRoutine indexRollerSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Indexer/Roller/SysID State", state.toString())),
          new Mechanism((volts) -> spinnerIO.setRollerVoltage(volts.in(Volts)), null, this));

  private SysIdRoutine kickerSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Indexer/Kicker/SysID State", state.toString())),
          new Mechanism((volts) -> kickerIO.setRollerVoltage(volts.in(Volts)), null, this));

  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 10.0;

  private LoggedTunableNumber testKickVolts = new LoggedTunableNumber("Indexer/Kicker Voltage", 10);
  private LoggedTunableNumber testSpinVolts = new LoggedTunableNumber("Indexer/Spinner Voltage", 8);

  private final Alert spinnerDisconnectedAlert =
      new Alert("Disconnected spinner motor!", AlertType.kError);
  private final Alert kickerDisconnectedAlert =
      new Alert("Disconnected kicker motor!", AlertType.kError);

  public SpindexerSubsystem(CANBus canbus, RollerIO indexRollerIO, RollerIO kickerIO) {
    this.kickerIO = kickerIO;
    this.spinnerIO = indexRollerIO;
  }

  @Override
  public Command index() {
    return this.run(
        () -> {
          spinnerIO.setRollerVoltage(7);
          kickerIO.setRollerVoltage(-7);
        });
  }

  @Override
  public Command kick() {
    return this.run(
        () -> {
          // spinnerIO.setRollerVoltage(12);
          spinnerIO.setRollerVelocity(18);
          // kickerIO.setRollerVoltage(11);
          kickerIO.setRollerVelocity(60);
        });
  }

  @Override
  public Command spit() {
    return this.run(
        () -> {
          spinnerIO.setRollerVoltage(-7);
          kickerIO.setRollerVoltage(-7);
        });
  }

  @Override
  public Command rest() {
    return this.run(
        () -> {
          spinnerIO.setRollerVoltage(0.0);
          kickerIO.setRollerVoltage(0.0);
        });
  }

  public static TalonFXConfiguration getIndexerConfigs() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = SPINNER_GEAR_RATIO;

    config.Slot0.kS = 0.25181;
    config.Slot0.kV = 0.66739;
    config.Slot0.kA = 0.038125;
    config.Slot0.kP = 0.1;
    config.Slot0.kD = 0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    return config;
  }

  public static TalonFXConfiguration getKickerConfigs() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Converts angular motion to linear motion
    config.Feedback.SensorToMechanismRatio = KICKER_GEAR_RATIO;

    config.Slot0.kS = 0.22251;
    config.Slot0.kV = 0.17199;
    config.Slot0.kA = 0.024802;
    config.Slot0.kP = 7;
    config.Slot0.kD = 0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    return config;
  }

  @Override
  public void periodic() {
    spinnerIO.updateInputs(spinnerInputs);
    Logger.processInputs("Indexer/Roller", spinnerInputs);
    kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Indexer/Kicker", kickerInputs);
    spinnerDisconnectedAlert.set(!spinnerInputs.connected);
    kickerDisconnectedAlert.set(!kickerInputs.connected);
  }

  public Command runRollerSysId() {
    return Commands.sequence(
        indexRollerSysid.quasistatic(Direction.kForward),
        indexRollerSysid.quasistatic(Direction.kReverse),
        indexRollerSysid.dynamic(Direction.kForward),
        indexRollerSysid.dynamic(Direction.kReverse));
  }

  @Override
  public Command runKickerSysId() {
    return Commands.sequence(
        kickerSysid.quasistatic(Direction.kForward),
        kickerSysid.quasistatic(Direction.kReverse),
        kickerSysid.dynamic(Direction.kForward),
        kickerSysid.dynamic(Direction.kReverse));
  }

  @Override
  public Command testShoot() {
    return this.run(
        () -> {
          kickerIO.setRollerVoltage(testKickVolts.get());
          spinnerIO.setRollerVoltage(testSpinVolts.get());
        });
  }
}
