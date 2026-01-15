package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import org.littletonrobotics.junction.Logger;

public class PivotIOReal {
  @AutoLog
  public static class PivotIOInputs {
    public double angularVelocityRotsPerSec = 0.0;
    public Rotation2d position = new Rotation2d();
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
    public double motorTemperatureCelsius = 0.0;
  }

  protected final TalonFX motor;
  private final String name;

  private final StatusSignal<AngularVelocity> angularVelocityRotsPerSec;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Angle> motorPositionRotations;
  private final StatusSignal<Temperature> motorTemperatureCelsius;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);

  public PivotIOReal(int motorID, TalonFXConfiguration config, String name) {
    motor = new TalonFX(motorID, "*");
    this.name = name;

    angularVelocityRotsPerSec = motor.getVelocity();
    supplyCurrentAmps = motor.getSupplyCurrent();
    statorCurrentAmps = motor.getStatorCurrent();
    appliedVoltage = motor.getMotorVoltage();
    motorPositionRotations = motor.getPosition();
    motorTemperatureCelsius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        angularVelocityRotsPerSec,
        supplyCurrentAmps,
        statorCurrentAmps,
        appliedVoltage,
        motorPositionRotations,
        motorTemperatureCelsius);

    motor.getConfigurator().apply(config);
    motor.optimizeBusUtilization();
  }

  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRotsPerSec,
        motorPositionRotations,
        supplyCurrentAmps,
        statorCurrentAmps,
        appliedVoltage,
        motorTemperatureCelsius);

    inputs.angularVelocityRotsPerSec = angularVelocityRotsPerSec.getValueAsDouble();
    inputs.position = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.motorTemperatureCelsius = motorTemperatureCelsius.getValueAsDouble();
  }

  public void setMotorVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  public void setMotorPosition(Rotation2d targetPosition) {
    // motor.setControl(motionMagic.withPosition(targetPosition.getRotations()).withSlot(slot));
    // clamp to avoid dead zone
    // 0 is forward
    // assume dead zone is back right corner i guess
    Rotation2d clampedPosition =
        Rotation2d.fromRadians(
            MathUtil.clamp(targetPosition.getRadians(), Math.PI / 2, -Math.PI / 4));
    Logger.recordOutput(name + "/Clamped Pivot Setpoint/", clampedPosition);
    motor.setControl(positionVoltage.withPosition(clampedPosition.getRotations()));
  }

  public void resetEncoder(Rotation2d rotations) {
    motor.setPosition(rotations.getRotations());
  }

  public static PivotIOReal getTurretPivotReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kV = 0.0;
    config.Slot0.kG = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = 1.0;

    return new PivotIOReal(0, config, "Pivot");
  }
}
