package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public class TurretIO {
  public static double TURRET_GEAR_RATIO = (42.0 / 12.0) * (32.0 / 16.0) * (85.0 / 10.0);

  public static double CANCODER_24T_TO_TURRET_GEAR_RATIO = (24.0 / 32.0) * (10.0 / 85.0);
  public static double CANCODER_26T_TO_TURRET_GEAR_RATIO = (26.0 / 32.0) * (10.0 / 85.0);

  // idk
  public static Rotation2d TURRET_MIN_ROTATIONS = Rotation2d.fromRotations(0.0);
  public static Rotation2d TURRET_MAX_ROTATIONS = Rotation2d.fromRotations(0.8);

  protected final TalonFX motor;

  @AutoLog
  public static class TurretIOInputs {
    public double angularVelocityRotationsPerSec = 0.0;
    public Rotation2d positionRotations = new Rotation2d();
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmp = 0.0;
    public double voltage = 0.0;
    public double tempCelsius = 0.0;
  }

  private final StatusSignal<AngularVelocity> angularVelocityRotationsPerSec;
  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Temperature> tempCelcius;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0);

  // todo
  private Rotation2d turretSetpoint = Rotation2d.kZero;

  public TurretIO(CANBus canivore) {
    motor = new TalonFX(15, canivore);
    final TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;

    config.Slot0.kS = 0;
    config.Slot0.kG = 0;
    config.Slot0.kV = 0;
    config.Slot0.kP = 0;
    config.Slot0.kD = 0;

    motor.getConfigurator().apply(config);

    angularVelocityRotationsPerSec = motor.getVelocity();
    positionRotations = motor.getPosition();
    supplyCurrentAmps = motor.getSupplyCurrent();
    statorCurrentAmps = motor.getStatorCurrent();
    voltage = motor.getMotorVoltage();
    tempCelcius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        angularVelocityRotationsPerSec,
        positionRotations,
        voltage,
        statorCurrentAmps,
        supplyCurrentAmps,
        tempCelcius);
    motor.optimizeBusUtilization();
  }

  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionRotations,
        angularVelocityRotationsPerSec,
        voltage,
        statorCurrentAmps,
        supplyCurrentAmps,
        tempCelcius);

    inputs.positionRotations = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.angularVelocityRotationsPerSec = angularVelocityRotationsPerSec.getValueAsDouble();
    inputs.voltage = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmp = supplyCurrentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelcius.getValueAsDouble();
  }

  public void setTurretPosition(Rotation2d positionAngle) {
    turretSetpoint = positionAngle;
    motor.setControl(
        motionMagic.withPosition(
            MathUtil.clamp(
                positionAngle.getRotations(),
                TURRET_MIN_ROTATIONS.getRotations(),
                TURRET_MAX_ROTATIONS.getRotations())));
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
