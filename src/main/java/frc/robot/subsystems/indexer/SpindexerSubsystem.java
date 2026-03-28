package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Spindexer = Spinning Indexer. !! COMP !! */
public class SpindexerSubsystem extends SubsystemBase implements Indexer {

  public static final double SPINNER_GEAR_RATIO = 67.0 / 15.0;
  public static final double KICKER_GEAR_RATIO = 24.0 / 18.0;
  // i don't really know if i should be using the sushi or the stealth wheels but the sushi wheels
  // are 1" in diameter and the stealth wheels are 3" in diameter
  public static final double KICKER_DIAMETER_INCHES = 3;
  // biggest wheel (smallest wheel is 2")
  public static final double SPINNER_DIAMETER_INCHES = 8;

  private RollerIO spinnerIO;

  private RollerIOInputsAutoLogged spinnerInputs = new RollerIOInputsAutoLogged();

  private RollerIO kickerIO;
  private RollerIOInputsAutoLogged kickerInputs = new RollerIOInputsAutoLogged();

  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 10.0;

  private final Alert spinnerDisconnectedAlert =
      new Alert("Disconnected spinner motor!", AlertType.kError);
  private final Alert kickerDisconnectedAlert =
      new Alert("Disconnected kicker motor!", AlertType.kError);

  @AutoLogOutput(key = "Kicker/Current Filter Value")
  private double currentFilterValue = 0.0;

  private LinearFilter kickerCurrentFilter = LinearFilter.movingAverage(5);

  public static final double KICKER_CURRENT_THRESHOLD = 20; // TODO

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
    return Commands.sequence(
        this.run(
            () -> {
              //   spinnerIO.setRollerVoltage(12);
              //   kickerIO.setRollerVoltage(12);
              // })
              //     .withTimeout(3),
              // this.run(
              //     () -> {
              spinnerIO.setRollerVelocity(30);
              kickerIO.setRollerVelocity(40);
            }));
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

  @Override
  public void periodic() {
    spinnerIO.updateInputs(spinnerInputs);
    Logger.processInputs("Indexer/Spinner", spinnerInputs);
    kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Indexer/Kicker", kickerInputs);
    spinnerDisconnectedAlert.set(!spinnerInputs.connected);
    kickerDisconnectedAlert.set(!kickerInputs.connected);

    currentFilterValue = kickerCurrentFilter.calculate(kickerInputs.statorCurrentAmps);
  }

  public static TalonFXConfiguration getIndexerConfig() {
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
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    return config;
  }

  public static TalonFXConfiguration getKickerConfig() {
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
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    return config;
  }

  public boolean isEmpty() {
    return currentFilterValue < KICKER_CURRENT_THRESHOLD;
  }
}
