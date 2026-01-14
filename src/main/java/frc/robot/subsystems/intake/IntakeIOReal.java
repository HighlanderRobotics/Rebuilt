package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal extends IntakeSubsystem{
    
    public static class IntakeIOInputs {
        public double velocity = 0.0;
        public double voltage = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempC = 0.0;
    }
    // 
    protected TalonFX motor = new TalonFX(2); //TODO find device id
    public IntakeIOReal() {
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
    private final StatusSignal<statorCurrentAmps> statorCurrentAmps = motor.getStatorCurrent();
    private final StatusSignal<supplyCurrentAmps> supplyCurrentAmps = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> tempC = motor.getDeviceTemp();

    private VoltageOut = new VoltageOut(0.0).withEnableFOC(true);
    
    public IntakeIOReal() {
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, velocity, voltage, statorCurrentAmps, supplyCurrentAmps,tempC);
        motor.optimizeBusUtilization();
    }
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocity, voltage, statorCurrentAmps, supplyCurrentAmps, tempC);
        inputs.velocity = velocity.getValueAsDouble();
        inputs.voltage = voltage
    }
    }
}
