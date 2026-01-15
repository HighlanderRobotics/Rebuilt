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

public class IndexerSubsystem extends SubsystemBase {

  // Add actual CanBus

  CANBus CANBus = new CANBus();
  CANrangeIOReal firstCANRange = new CANrangeIOReal(0, CANBus);
  CANrangeIOReal secondCANRange = new CANrangeIOReal(1, CANBus);
  CANrangeIOInputsAutoLogged CANRangeInputs = new CANrangeIOInputsAutoLogged();
  TalonFXConfiguration configs = new TalonFXConfiguration();
  RollerIOReal rollers = new RollerIOReal(1, configs);

  RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  public IndexerSubsystem() {}

  public boolean isFull(boolean firstBeamBreak, boolean secondBeamBreak) {
    if (firstBeamBreak && secondBeamBreak) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isEmpty(boolean firstBeamBreak, boolean secondBeamBreak) {
    if (!firstBeamBreak && !secondBeamBreak) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isPartiallyFull(boolean firstBeamBreak, boolean secondBeamBreak) {
    if (!firstBeamBreak && secondBeamBreak) {
      return true;
    } else {
      return false;
    }
  }

  public Command index(DoubleSupplier volts) {
    return this.run(
        () -> {
          rollers.setRollerVoltage(volts.getAsDouble());
        });
  }

  public Command score(DoubleSupplier volts) {
    return this.run(
        () -> {
          rollers.setRollerVoltage(volts.getAsDouble());
        });
  }

  public Command outtake(DoubleSupplier volts) {
    return this.run(
        () -> {
          rollers.setRollerVoltage(volts.getAsDouble());
        });
  }

  @Override
  public void periodic() {
    firstCANRange.updateInputs(CANRangeInputs);
    secondCANRange.updateInputs(CANRangeInputs);
    rollers.updateInputs(rollerInputs);
  }
}
