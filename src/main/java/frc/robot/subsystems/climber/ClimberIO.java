package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double motorPositionMeters = 0.0;
    public double motorVelocityMetersPerSec = 0.0;
    public double motorStatorCurrentAmps = 0.0;
    public double motorSupplyCurrentAmps = 0.0;
    public double motorVoltage = 0.0;
    public double motorTempC = 0.0;
  }

  protected final TalonFX climberMotor;

  // Rotation -> linear conversion happens in sensor to mech ratio
  private final StatusSignal<Angle> motorPositionMeters;
  private final StatusSignal<AngularVelocity> velocityMetersPerSec;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Temperature> motorTemp;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

  private double climberSetpoint = 0.0;

  public ClimberIO(CANBus canBus) {
    // todo: set correct motor ID
    climberMotor = new TalonFX(30, canBus);
    climberMotor.getConfigurator().apply(ClimberIO.getClimberConfiguration());

    velocityMetersPerSec = climberMotor.getVelocity();
    supplyCurrentAmps = climberMotor.getSupplyCurrent();
    motorVoltage = climberMotor.getMotorVoltage();
    statorCurrentAmps = climberMotor.getStatorCurrent();
    motorTemp = climberMotor.getDeviceTemp();
    motorPositionMeters = climberMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        velocityMetersPerSec,
        supplyCurrentAmps,
        motorVoltage,
        statorCurrentAmps,
        motorTemp,
        motorPositionMeters);
    climberMotor.optimizeBusUtilization();
  }

  public static TalonFXConfiguration getClimberConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // todo: find and make climber gear ratio variable
    config.Feedback.SensorToMechanismRatio =
        ClimberSubsystem.GEAR_RATIO * (Math.PI * ClimberSubsystem.SPOOL_DIAMETER_METERS);

    // todo: tune
    config.Slot0.kS = 0.0;
    config.Slot0.kG = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kD = 0.0;

    // todo: find actual current limits
    config.CurrentLimits.StatorCurrentLimit = 50.00;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.00;

    return config;
  }

  public void setClimberPosition(double climberPosition) {
    climberSetpoint = climberPosition;
    climberMotor.setControl(positionVoltage.withPosition(climberSetpoint));
  }

  public void setClimberVoltage(double climberVoltage) {
    climberMotor.setControl(voltageOut.withOutput(climberVoltage));
  }

  public void setClimberVelocity(double climberVelocity) {
    climberMotor.setControl(velocityVoltage.withVelocity(climberVelocity));
  }

  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorPositionMeters,
        velocityMetersPerSec,
        statorCurrentAmps,
        supplyCurrentAmps,
        motorVoltage,
        motorTemp);

    inputs.motorPositionMeters = motorPositionMeters.getValueAsDouble();
    inputs.motorVoltage = motorVoltage.getValueAsDouble();
    inputs.motorTempC = motorTemp.getValueAsDouble();
    inputs.motorSupplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.motorStatorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.motorVelocityMetersPerSec = velocityMetersPerSec.getValueAsDouble();
  }

  @AutoLogOutput(key = "Climber/Setpoint Meters")
  public double getClimberSetpointMeters() {
    return climberSetpoint;
  }
}
