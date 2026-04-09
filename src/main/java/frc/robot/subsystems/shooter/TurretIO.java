package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public class TurretIO {

  protected final TalonFX motor;

  @AutoLog
  public static class TurretIOInputs {
    public double angularVelocityRotationsPerSec = 0.0;
    public Rotation2d positionRotations = new Rotation2d();
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmp = 0.0;
    public double voltage = 0.0;
    public double tempCelsius = 0.0;
    public boolean connected = false;
  }

  private final StatusSignal<AngularVelocity> angularVelocityRotationsPerSec;
  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Temperature> tempC;

  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);

  // todo
  private Rotation2d turretSetpoint = Rotation2d.kZero;

  public TurretIO(CANBus canivore, TalonFXConfiguration config) {
    motor = new TalonFX(15, canivore);
    motor.getConfigurator().apply(config);

    angularVelocityRotationsPerSec = motor.getVelocity();
    positionRotations = motor.getPosition();
    supplyCurrentAmps = motor.getSupplyCurrent();
    statorCurrentAmps = motor.getStatorCurrent();
    voltage = motor.getMotorVoltage();
    tempC = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        angularVelocityRotationsPerSec,
        positionRotations,
        voltage,
        statorCurrentAmps,
        supplyCurrentAmps,
        tempC);
    motor.optimizeBusUtilization();
  }

  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionRotations,
        angularVelocityRotationsPerSec,
        voltage,
        statorCurrentAmps,
        supplyCurrentAmps,
        tempC);

    inputs.connected =
        BaseStatusSignal.isAllGood(
            positionRotations,
            angularVelocityRotationsPerSec,
            voltage,
            statorCurrentAmps,
            supplyCurrentAmps,
            tempC);
    inputs.positionRotations = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.angularVelocityRotationsPerSec = angularVelocityRotationsPerSec.getValueAsDouble();
    inputs.voltage = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmp = supplyCurrentAmps.getValueAsDouble();
    inputs.tempCelsius = tempC.getValueAsDouble();
  }

  public void setTurretPosition(Rotation2d positionAngle) {
    turretSetpoint = positionAngle;
    // motor.setControl(motionMagic.withPosition(positionAngle.getRotations()));
    motor.setControl(positionVoltage.withPosition(positionAngle.getRotations()));
  }

  public void resetTurretEncoder(Rotation2d turretRotation) {
    motor.setPosition(turretRotation.getRotations());
  }

  @AutoLogOutput(key = "Shooter/Turret/Setpoint")
  public Rotation2d getTurretSetpoint() {
    return turretSetpoint;
  }

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
