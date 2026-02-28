package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public class LinearRackIO {

  @AutoLog
  public static class LinearRackIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSecond = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double temperatureC = 0.0;
    public boolean connected = false;
  }

  protected final TalonFX motor;

  // Technically, these measure angle, but the conversion from angle to linear movement happens in
  // the sensor-to-mechanism ratio
  private final StatusSignal<Angle> positionMeters;
  private final StatusSignal<AngularVelocity> velocityMetersPerSecond;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Temperature> temperatureC;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  // I think we might want to motion profile this so i'm using motion magic
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true);

  private double setpointMeters = 0.0;

  public LinearRackIO(int motorID, CANBus canBus, TalonFXConfiguration config) {
    this.motor = new TalonFX(motorID, canBus);

    positionMeters = motor.getPosition();
    velocityMetersPerSecond = motor.getVelocity();
    appliedVoltage = motor.getMotorVoltage();
    statorCurrentAmps = motor.getStatorCurrent();
    supplyCurrentAmps = motor.getSupplyCurrent();
    temperatureC = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionMeters,
        velocityMetersPerSecond,
        appliedVoltage,
        statorCurrentAmps,
        supplyCurrentAmps,
        temperatureC);
    motor.optimizeBusUtilization();

    motor.getConfigurator().apply(config);
  }

  public void updateInputs(LinearRackIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionMeters,
        velocityMetersPerSecond,
        appliedVoltage,
        statorCurrentAmps,
        supplyCurrentAmps,
        temperatureC);
    inputs.connected =
        BaseStatusSignal.isAllGood(
            positionMeters,
            velocityMetersPerSecond,
            appliedVoltage,
            statorCurrentAmps,
            supplyCurrentAmps,
            temperatureC);
    inputs.positionMeters = positionMeters.getValueAsDouble();
    inputs.velocityMetersPerSecond = velocityMetersPerSecond.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.temperatureC = temperatureC.getValueAsDouble();
  }

  public void setVoltage(double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }

  public void setPositionSetpoint(double setpointMeters) {
    this.setpointMeters = setpointMeters;
    motor.setControl(motionMagicVoltage.withPosition(setpointMeters));
  }

  @AutoLogOutput(key = "Intake/Setpoint")
  public double getSetpointMeters() {
    return setpointMeters;
  }

  public void resetEncoder(double positionMeters) {
    motor.setPosition(positionMeters);
  }
}
