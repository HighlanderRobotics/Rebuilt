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
import edu.wpi.first.math.geometry.Rotation2d;
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
  // shared gear rotations per motor turn
  public static double MOTOR_TO_SHARED_GEAR_GEAR_RATIO = (12 / 42) * (16 / 32);
  // turret rotations per shared gear rotations:
  public static double CANCODER_SHARED_GEAR_TO_TURRET_GEAR_RATIO = 10.0 / 85.0;

  // shared gear roations per encoder rotations
  public static double CANCODER_ONE_GEAR_RATIO = 24.0 / 32.0;
  public static double CANCODER_TWO_GEAR_RATIO = 26.0 / 32.0;

  // idk
  public static double TURRET_MIN_ROTATIONS = 0.0;
  public static double TURRET_MAX_ROTATIONS = 0.8;

  // todo ID?
  protected final TalonFX motor = new TalonFX(40, "*");
  private final CANcoderIO cancoderOne;
  private final CANcoderIO cancoderTwo;
  private final CANcoderIOInputsAutoLogged cancoderOneInputs = new CANcoderIOInputsAutoLogged();
  private final CANcoderIOInputsAutoLogged cancoderTwoInputs = new CANcoderIOInputsAutoLogged();

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
  private Rotation2d turretAbsolutePos = getAbsoluteTurretPosition();

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

  public void setPositionAngle(Rotation2d positionAngle) {
    turretSetpoint = positionAngle;
    motor.setControl(positionVoltage.withPosition(positionAngle.getRotations()));
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
  }

  @AutoLogOutput(key = "Shooter/Turret/Setpoint")
  public Rotation2d getTurretSetpoint() {
    return turretSetpoint;
  }

  public static CANcoderConfiguration getCancoderConfigs() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = 0.0;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;

    return config;
  }

  // todo
  public void resetTurretPosition(Rotation2d turretRotation) {
    // uhh is this right
    // clamp between max and min bc it can only go so much
    motor.setPosition(
        Math.min(
            Math.max(turretRotation.getRotations(), TURRET_MIN_ROTATIONS), TURRET_MAX_ROTATIONS));
  }

  // feels dangerously simple
  public Rotation2d getAbsoluteTurretPosition() {
    Rotation2d cancoder1 = cancoderOneInputs.cancoderPositionRotations;
    Rotation2d cancoder2 = cancoderTwoInputs.cancoderPositionRotations;

    // find difference and wrap to -0.5 and 0.5
    // this is bc diff wont exceed 1 and we want it to show like which direction it is
    double diffRotations =
        cancoder1.getRotations() - cancoder2.getRotations(); // modulus thing , -0.5, 0.5);

    // round bc we only want the full rotations i think
    // actually im not sure the rounding is nessicary i think we can just find it directly but i
    // could be wrong
    // double fullRotations = round(diffRotations * 32 / (26 - 24));

    // 32/(26-24) gearing difference repeats every 16
    // so a full rotation of the shared gear is the difference times 16
    // get shared gear rotations:

    // don't ask me how i got this number
    double absoluteRotations = diffRotations * (24 * 26) / (32 * 2);

    // total rotations by adding full rotations to the position you are on that rotation
    // double absoluteRotations = fullRotations + sharedGearFromCan1.getRotations();

    // get turret by using absolute rotations times the cancoder shared gear
    double turretRotations = absoluteRotations * CANCODER_SHARED_GEAR_TO_TURRET_GEAR_RATIO;

    return Rotation2d.fromRotations(turretRotations);
  }
}
