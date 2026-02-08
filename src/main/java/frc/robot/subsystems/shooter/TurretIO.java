package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.components.cancoder.CANcoderIO;
import frc.robot.components.cancoder.CANcoderIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public class TurretIO {
  public static double TURRET_GEAR_RATIO = (42.0 / 12.0) * (32.0 / 16.0) * (85.0 / 10.0);

  public static double CANCODER_ONE_TO_TURRET_GEAR_RATIO = (24.0 / 32.0) * (10.0 / 85.0);

  // idk
  public static Rotation2d TURRET_MIN_ROTATIONS = Rotation2d.fromRotations(0.0);
  public static Rotation2d TURRET_MAX_ROTATIONS = Rotation2d.fromRotations(0.8);

  // todo ID?
  protected final TalonFX motor = new TalonFX(40, "*");
  private final CANcoderIO cancoderOne;
  private final CANcoderIO cancoderTwo;
  private final CANcoderIOInputsAutoLogged cancoderOneInputs = new CANcoderIOInputsAutoLogged();
  private final CANcoderIOInputsAutoLogged cancoderTwoInputs = new CANcoderIOInputsAutoLogged();

  @AutoLogOutput(key = "Shooter/Turret/Setpoint")
  public Rotation2d getTurretSetpoint() {
    return turretSetpoint;
  }

  @AutoLogOutput(key = "Shooter/Turret/Cancoder One")
  public Rotation2d getTurretCancoderOne() {
    return cancoderOneInputs.cancoderPositionRotations;
  }

  @AutoLogOutput(key = "Shooter/Turret/Cancoder Two")
  public Rotation2d getTurretCancoderTwo() {
    return cancoderOneInputs.cancoderPositionRotations;
  }

  @AutoLogOutput(key = "Shooter/Turret/Turret Rotation")
  public Rotation2d getTurretRotation() {
    return getAbsoluteTurretRotations();
  }

  @AutoLog
  public static class TurretIOInputs {
    public double angularVelocityRotationsPerSec = 0.0;
    public Rotation2d positionRotations = new Rotation2d();
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmp = 0.0;
    public double voltage = 0.0;
    public double tempCelcius = 0.0;
  }

  private final StatusSignal<AngularVelocity> angularVelocityRotationsPerSec = motor.getVelocity();
  private final StatusSignal<Angle> positionRotations = motor.getPosition();
  private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
  private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Temperature> tempCelcius = motor.getDeviceTemp();

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

  // todo
  private Rotation2d turretSetpoint = Rotation2d.kZero;

  public TurretIO(CANcoderIO cancoderOne, CANcoderIO cancoderTwo) {
    this.cancoderOne = cancoderOne;
    this.cancoderTwo = cancoderTwo;

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
    inputs.tempCelcius = tempCelcius.getValueAsDouble();
    cancoderOne.updateInputs(cancoderOneInputs);
    cancoderTwo.updateInputs(cancoderTwoInputs);
  }

  public static CANcoderConfiguration getCancoderConfigs() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = 0.0;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;

    return config;
  }

  public void setTurretPosition(Rotation2d positionAngle) {
    turretSetpoint = positionAngle;
    motor.setControl(
        positionVoltage.withPosition(
            Math.min(
                Math.max(positionAngle.getRotations(), TURRET_MIN_ROTATIONS.getRotations()),
                TURRET_MAX_ROTATIONS.getRotations())));
  }

  public void resetTurretPosition(Rotation2d turretRotation) {
    motor.setPosition(turretRotation.getRotations());
  }

  public Rotation2d getAbsoluteTurretRotations() {
    // give valaues between 0 and 1
    Rotation2d cancoder1 = cancoderOneInputs.cancoderPositionRotations;
    Rotation2d cancoder2 = cancoderTwoInputs.cancoderPositionRotations;

    // if can one is bigger than can 2 its simply can1-can2
    // otherwise can1 + 1 - can2 because we want how much behind can1 it is
    double diffRotations =
        (cancoder1.getRotations() >= cancoder2.getRotations())
            ? cancoder1.getRotations() - cancoder2.getRotations()
            : (cancoder1.getRotations() + 1) - cancoder2.getRotations();

    // keeping track of how many total rots can1 is doing using the diff with can 2
    // which is just diff times 26/2 because every 13 turns they both reach some full amount of rots
    // bc gear ratio
    double absoluteRotationsCan1 = diffRotations * (26.0 / 2.0);

    // turret maxes out at like 11 can 1 rotations anyways so it should work up to there and i
    // tested
    // multiply abs can1 rots by the gear ratio
    double turretRotations = absoluteRotationsCan1 * TurretIO.CANCODER_ONE_TO_TURRET_GEAR_RATIO;

    return Rotation2d.fromRotations(turretRotations);
  }
}
