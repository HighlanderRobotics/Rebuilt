package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;

import frc.robot.components.canrange.CANrangeIOInputsAutoLogged;
import frc.robot.components.canrange.CANrangeIOReal;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

//Add actual CanBus

CANBus CANBus = new CANBus();
CANrangeIOReal firstCANRange = new CANrangeIOReal(0, CANBus);
CANrangeIOReal secondCANRange = new CANrangeIOReal(1, CANBus);
CANrangeIOInputsAutoLogged inputs = new CANrangeIOInputsAutoLogged();
TalonFX motor = new TalonFX(1);
VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
RollerIO rollers = new RollerIO();
    
    public IndexerSubsystem(){

    }
    

    public void index(double volts) {
        while(!isFull(firstCANRange, secondCANRange)){
            motor.setControl(voltageOut.withOutput(volts));
        }
        motor.setControl(voltageOut.withOutput(0));
        
    }

    public void score(double volts) {
        motor.setControl(voltageOut.withOutput(volts));
    }

    public void outtake(double volts) {
        motor.setControl(voltageOut.withOutput(volts));
    }

    public boolean isFull(boolean firstBeamBreak, boolean secondBeamBreak) {
        if(firstBeamBreak && secondBeamBreak){
            return true;
        } else {
            return false;
        }
    }

    public boolean isEmpty(boolean firstBeamBreak, boolean secondBeamBreak) {
        if(!firstBeamBreak && !secondBeamBreak){
            return true;
        } else {
            return false;
        }
    }

    public boolean isPartiallyFull(boolean firstBeamBreak, boolean secondBeamBreak) {
        if(!firstBeamBreak && secondBeamBreak){
            return true;
        } else {
            return false;
        }

    }
    public Command index(DoubleSupplier volts) {
    return this.run(
        () -> {
          index(volts.getAsDouble());
        });
  }

    public Command score(DoubleSupplier volts) {
    return this.run(
        () -> {
          score(volts.getAsDouble());
        });
  }

    public Command outtake(DoubleSupplier volts) {
    return this.run(
        () -> {
          outtake(volts.getAsDouble());
        });
  }

@Override
  public void periodic() {
    firstCANRange.updateInputs(inputs);
    secondCANRange.updateInputs(inputs);

  }
    
}
