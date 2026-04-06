package frc.robot.components.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class RollerIO {

  @AutoLog
  public static class RollerIOInputs {
    public double velocityRotsPerSec = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double positionRotations = 0.0;
    public boolean connected = false;
  }

  protected final TalonFX motor;

  private final StatusSignal<AngularVelocity> angularVelocityRotsPerSec;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Temperature> temperatureCelsius;
  private final StatusSignal<Angle> positionRotations;

  private double setpoint;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

  public RollerIO(int motorID, TalonFXConfiguration config, CANBus canbus) {
    // it's telling me this leads to heap pollution...which is probably unfortunate but i don't
    // think that will happen!
    motor = new TalonFX(motorID, canbus);

    angularVelocityRotsPerSec = motor.getVelocity();
    supplyCurrentAmps = motor.getSupplyCurrent();
    appliedVoltage = motor.getMotorVoltage();
    statorCurrentAmps = motor.getStatorCurrent();
    temperatureCelsius = motor.getDeviceTemp();
    positionRotations = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        angularVelocityRotsPerSec,
        supplyCurrentAmps,
        statorCurrentAmps,
        appliedVoltage,
        temperatureCelsius,
        positionRotations);

    motor.getConfigurator().apply(config);
    motor.optimizeBusUtilization();
  }

  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRotsPerSec,
        supplyCurrentAmps,
        appliedVoltage,
        statorCurrentAmps,
        temperatureCelsius,
        positionRotations);

    inputs.connected =
        BaseStatusSignal.isAllGood(
            angularVelocityRotsPerSec,
            supplyCurrentAmps,
            appliedVoltage,
            statorCurrentAmps,
            temperatureCelsius,
            positionRotations);
    inputs.velocityRotsPerSec = angularVelocityRotsPerSec.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
    inputs.positionRotations = positionRotations.getValueAsDouble();
  }

  public void setRollerVoltage(double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }

  public void setRollerVelocity(double velocityRPS) {
    setpoint = velocityRPS;
    motor.setControl(velocityVoltage.withVelocity(velocityRPS));
  }

  public double getVelocitySetpoint() {
    return setpoint;
  }
}
