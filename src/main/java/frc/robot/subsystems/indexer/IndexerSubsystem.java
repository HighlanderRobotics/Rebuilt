package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.canrange.CANrangeIOInputsAutoLogged;
import frc.robot.components.canrange.CANrangeIOReal;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
  private CANrangeIOReal firstCANRangeIO;
  private CANrangeIOReal secondCANRangeIO;

  private RollerIO rollerIO;

  private CANrangeIOInputsAutoLogged firstCANRangeInputs = new CANrangeIOInputsAutoLogged();
  private CANrangeIOInputsAutoLogged secondCANRangeInputs = new CANrangeIOInputsAutoLogged();

  private RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 10.0;

  public IndexerSubsystem(CANBus canbus, RollerIO rollerIO) {
    firstCANRangeIO = new CANrangeIOReal(0, canbus);
    secondCANRangeIO = new CANrangeIOReal(1, canbus);
    this.rollerIO = rollerIO;
  }

  public boolean isFull() {
    return firstCANRangeInputs.isDetected && secondCANRangeInputs.isDetected;
  }

  public boolean isEmpty() {
    return !firstCANRangeInputs.isDetected && !secondCANRangeInputs.isDetected;
  }

  public boolean isPartiallyFull() {
    return !firstCANRangeInputs.isDetected && secondCANRangeInputs.isDetected;
  }

  public Command index() {
    return this.run(() -> rollerIO.setRollerVoltage(5));
  }

  public Command score() {
    return this.run(() -> rollerIO.setRollerVoltage(10));
  }

  public Command outtake() {
    return this.run(() -> rollerIO.setRollerVoltage(-5));
  }

  public static TalonFXConfiguration getIndexerConfigs() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Converts angular motion to linear motion
    config.Feedback.SensorToMechanismRatio = 1;

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
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Indexer/Roller", rollerInputs);
  }
}
