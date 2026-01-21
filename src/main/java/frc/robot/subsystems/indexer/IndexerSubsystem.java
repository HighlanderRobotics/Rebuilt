package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*;
import frc.robot.components.canrange.CANrangeIOInputsAutoLogged;
import frc.robot.components.canrange.CANrangeIOReal;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

  public static final double GEAR_RATIO = 2.0;
  private CANrangeIOReal firstCANRangeIO;
  private CANrangeIOReal secondCANRangeIO;

  private RollerIO indexRollerIO;

  CANrangeIOInputsAutoLogged firstCANRangeInputs = new CANrangeIOInputsAutoLogged();
  CANrangeIOInputsAutoLogged secondCANRangeInputs = new CANrangeIOInputsAutoLogged();

  RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  RollerIO kickerIO;
  RollerIOInputsAutoLogged kickerInputs = new RollerIOInputsAutoLogged();

  private SysIdRoutine indexRollerSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Indexer/Roller/SysID State", state.toString())),
          new Mechanism((volts) -> indexRollerIO.setRollerVoltage(volts.in(Volts)), null, this));

  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 10.0;
  public static final double KICKER_GEAR_RATIO = 2.0;

  public IndexerSubsystem(CANBus canbus, RollerIO indexRollerIO, RollerIO kickerIO) {
    this.kickerIO = kickerIO;
    firstCANRangeIO = new CANrangeIOReal(0, canbus, 10);
    secondCANRangeIO = new CANrangeIOReal(1, canbus, 10);
    this.indexRollerIO = indexRollerIO;
  }

  public boolean isFull() {
    return firstCANRangeInputs.isDetected && secondCANRangeInputs.isDetected;
  }

  public Command stopKicker() {
    return this.run(() -> kickerIO.setRollerVoltage(0));
  }
  ;

  public Command shoot() {
    return this.run(() -> kickerIO.setRollerVoltage(0));
  }

  public boolean isEmpty() {
    return !firstCANRangeInputs.isDetected && !secondCANRangeInputs.isDetected;
  }

  public boolean isPartiallyFull() {
    return !firstCANRangeInputs.isDetected && secondCANRangeInputs.isDetected;
  }

  public Command index() {
    return this.run(
        () -> {
          indexRollerIO.setRollerVoltage(new LoggedTunableNumber("Indexer Roller/Index", 7).get());
          kickerIO.setRollerVoltage(new LoggedTunableNumber("Kicker/Index", -7).get());
        });
  }

  public Command kick() {
    return this.run(
        () -> {
          indexRollerIO.setRollerVoltage(new LoggedTunableNumber("Indexer Roller/Kick", 10).get());
          kickerIO.setRollerVoltage(new LoggedTunableNumber("Kicker/Kick", 5).get());
        });
  }

  public Command spit() {
    return this.run(
        () -> {
          indexRollerIO.setRollerVoltage(new LoggedTunableNumber("Indexer Roller/Spit", -5).get());
          kickerIO.setRollerVoltage(new LoggedTunableNumber("Kicker/Spit", -5).get());
        });
  }

  public Command rest() {
    return this.run(
        () -> {
          indexRollerIO.setRollerVoltage(0.0);
          kickerIO.setRollerVoltage(0.0);
        });
  }

  public static TalonFXConfiguration getIndexerConfigs() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.Slot0.kS = 0;
    config.Slot0.kG = 0;
    config.Slot0.kV = 0;
    config.Slot0.kP = 0;
    config.Slot0.kD = 0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    return config;
  }

  public static TalonFXConfiguration getKickerConfigs() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Converts angular motion to linear motion
    config.Feedback.SensorToMechanismRatio = KICKER_GEAR_RATIO;

    config.Slot0.kS = 0;
    config.Slot0.kG = 0;
    config.Slot0.kV = 0;
    config.Slot0.kP = 0;
    config.Slot0.kD = 0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    return config;
  }

  @Override
  public void periodic() {
    firstCANRangeIO.updateInputs(firstCANRangeInputs);
    Logger.processInputs("Indexer/First Beambreak", firstCANRangeInputs);
    secondCANRangeIO.updateInputs(secondCANRangeInputs);
    Logger.processInputs("Indexer/Second Beambreak", secondCANRangeInputs);
    indexRollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Indexer/Roller", rollerInputs);
    kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Indexer/Kicker", kickerInputs);
  }

  public Command runRollerSysId() {
    return Commands.sequence(
        indexRollerSysid.quasistatic(Direction.kForward),
        indexRollerSysid.quasistatic(Direction.kReverse),
        indexRollerSysid.dynamic(Direction.kForward),
        indexRollerSysid.dynamic(Direction.kReverse));
  }
}
