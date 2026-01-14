package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    
    public enum IntakeState{
        private IntakeSubsystem io;
        private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

        public void IntakeSubsystem(IntakeIOReal io){
            this.io = io;


    }
    }


    public Command setVoltage(DoubleSupplier volts) {
        return this.run(
            () -> {

                io.setVoltage(volts.getAsDouble());
            });
    }
    public Command getVoltage() {
        return io.getVoltage();
    }
}