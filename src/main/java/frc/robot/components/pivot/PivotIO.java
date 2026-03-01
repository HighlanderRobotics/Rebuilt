package frc.robot.components.pivot;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public Rotation2d position = new Rotation2d();
    public double angularVelocityRotationsPerSec = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double voltage = 0.0;
    public double tempC = 0.0;
    // For sysid
    public double positionRotations = 0.0;
    public boolean connected = false;
  }

  protected final TalonFX motor;

  private final StatusSignal<Angle> position;
  private final BaseStatusSignal angularVelocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private MotionMagicVoltage motorMagicVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true);

  private Rotation2d setpoint = Rotation2d.kZero;

  public PivotIO(int motorId, TalonFXConfiguration config, CANBus canBus) {
    motor = new TalonFX(motorId, canBus);
    motor.getConfigurator().apply(config);

    position = motor.getPosition();
    angularVelocity = motor.getVelocity();
    voltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, angularVelocity, voltage, statorCurrent, supplyCurrent, temp);
    motor.optimizeBusUtilization();
  }

  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, angularVelocity, voltage, statorCurrent, supplyCurrent, temp);

    inputs.connected = BaseStatusSignal.isAllGood(position, angularVelocity, voltage, statorCurrent, supplyCurrent, temp);

    inputs.position = new Rotation2d(position.getValue());
    inputs.positionRotations = position.getValue().in(Rotation);
    inputs.angularVelocityRotationsPerSec = angularVelocity.getValueAsDouble(); 
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempC = temp.getValueAsDouble();
  }

  public void setMotorVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  public void setMotorPositionSetpoint(Rotation2d setpoint) {
    this.setpoint = setpoint;
    motor.setControl(motorMagicVoltage.withPosition(setpoint.getMeasure()));
  }

  public Rotation2d getSetpoint() {
    return setpoint;
  }

  public void resetEncoder(Rotation2d newPosition) {
    motor.setPosition(newPosition.getMeasure());
  }
}
