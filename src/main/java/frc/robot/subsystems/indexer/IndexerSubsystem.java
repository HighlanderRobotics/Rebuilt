package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.canrange.CANrangeIOInputsAutoLogged;
import frc.robot.components.canrange.CANrangeIOReal;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import frc.robot.components.rollers.RollerIOReal;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

  // Add actual CanBus

    CANrangeIOReal firstCANRange;
    CANrangeIOReal secondCANRange;
    RollerIOReal rollerIO;
    CANrangeIOInputsAutoLogged firstCANRangeInputs = new CANrangeIOInputsAutoLogged();
    CANrangeIOInputsAutoLogged secondCANRangeInputs = new CANrangeIOInputsAutoLogged();
    RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  public IndexerSubsystem(CANBus canbus, RollerIOReal roller) {
    firstCANRange = new CANrangeIOReal(0, canbus);
    secondCANRange = new CANrangeIOReal(1, canbus);
    rollerIO = roller;

  }

  public boolean isFull() {
    
    return (firstCANRangeInputs.isDetected && secondCANRangeInputs.isDetected);

  }

  public boolean isEmpty() {
    return (!firstCANRangeInputs.isDetected && !secondCANRangeInputs.isDetected);
  }

  public boolean isPartiallyFull() {
    return (!firstCANRangeInputs.isDetected && secondCANRangeInputs.isDetected);
  }

  public Command index() {
    return this.run(
        () -> {
          rollerIO.setRollerVoltage(5);
        });
  }

  public Command score() {
    return this.run(
        () -> {
          rollerIO.setRollerVoltage(10);
        });
  }

  public Command outtake() {
    return this.run(
        () -> {
          rollerIO.setRollerVoltage(-5);
        });
  }

  @Override
  public void periodic() {
    firstCANRange.updateInputs(firstCANRangeInputs);
    Logger.processInputs("Indexer/First Beambreak", firstCANRangeInputs);
    secondCANRange.updateInputs(secondCANRangeInputs);
    Logger.processInputs("Indexer/Second Beambreak", secondCANRangeInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Indexer/Roller", rollerInputs);
  }
}
