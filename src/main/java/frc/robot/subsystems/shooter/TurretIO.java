package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.components.cancoder.CANcoderIO;
import frc.robot.components.cancoder.CANcoderIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public class TurretIO {
  public static double TURRET_GEAR_RATIO = (42.0 / 12.0) * (32.0 / 16.0) * (85.0 / 10.0);
  public static double CANCODER_SHARED_GEAR_TO_TURRET_GEAR_RATIO = 85.0 / 10.0;
  // dont think ts is right
  // public static double CANCODER_ONE_GEAR_RATIO = (42.0/12)*(24/16);
  //  public static double CANCODER_TWO_GEAR_RATIO = (42.0/12)*(26/16);

  // shared gear roations per encoder rotations
  public static double CANCODER_ONE_GEAR_RATIO = 24.0 / 32.0;
  public static double CANCODER_TWO_GEAR_RATIO = 26.0 / 32.0;

  //todo ID?
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

  // todo
  public void resetTurretPosition(Rotation2d turretRotation) {
    // uhh is this right
    motor.setPosition(turretRotation.getRotations());
  }

  public Rotation2d getAbsoluteTurretPosition() {
    // does this account for the gearing before the cancoders ig it wouldnt rlly matter nvm
    Rotation2d cancoder1 = cancoderOneInputs.cancoderPositionRotations;
    Rotation2d cancoder2 = cancoderTwoInputs.cancoderPositionRotations;

    Rotation2d sharedGearFromCan1 = cancoder1.times(CANCODER_ONE_GEAR_RATIO);
    Rotation2d sharedGearFromCan2 = cancoder2.times(CANCODER_TWO_GEAR_RATIO);

    // find difference and wrap to -0.5 and 0.5
    //bc diff wont exceed 1 and we want it to show like which way it is...
    //this is the part im less sure about
    double diffRotations =
        MathUtil.inputModulus(
            sharedGearFromCan2.getRotations() - sharedGearFromCan1.getRotations(), -0.5, 0.5);

    // 32/(26-24) = 16 so gearing difference repeats every 16
    // so a full rotation of the shared gear is the difference times 16
    // because after one rotation the difference is like 2t, then 4t etc
    // round bc only want the full rotations
    double fullRotations = Math.round(diffRotations * 32 / (26 - 24));

    // total roations by adding full rotations to the position you are on that rotation
    double absoluteRotations = fullRotations + sharedGearFromCan1.getRotations();

    // get turret by using absolute roations times the cancoder shared gear
    double turretRotations = absoluteRotations * CANCODER_SHARED_GEAR_TO_TURRET_GEAR_RATIO;

    return Rotation2d.fromRotations(turretRotations);
  }
}
