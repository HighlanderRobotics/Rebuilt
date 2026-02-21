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
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double tempC = 0.0;
    public boolean connected = false;
  }

  protected final TalonFX climberMotor;

  // Rotation -> linear conversion happens in sensor to mech ratio
  private final StatusSignal<Angle> positionMeters;
  private final StatusSignal<AngularVelocity> velocityMetersPerSec;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Temperature> tempC;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

  private double climberSetpoint = 0.0;

  public ClimberIO(CANBus canBus) {
    climberMotor = new TalonFX(16, canBus);
    climberMotor.getConfigurator().apply(ClimberIO.getClimberConfiguration());

    velocityMetersPerSec = climberMotor.getVelocity();
    supplyCurrentAmps = climberMotor.getSupplyCurrent();
    voltage = climberMotor.getMotorVoltage();
    statorCurrentAmps = climberMotor.getStatorCurrent();
    tempC = climberMotor.getDeviceTemp();
    positionMeters = climberMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        velocityMetersPerSec,
        supplyCurrentAmps,
        voltage,
        statorCurrentAmps,
        tempC,
        positionMeters);
    climberMotor.optimizeBusUtilization();
  }

  public static TalonFXConfiguration getClimberConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // after swapping it so it would be within extension limit the spooling reversed direction
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // todo: find and make climber gear ratio variable
    config.Feedback.SensorToMechanismRatio =
        ClimberSubsystem.GEAR_RATIO / (Math.PI * ClimberSubsystem.SPOOL_DIAMETER_METERS);

    config.Slot0.kP = 600.0;

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

  public void resetEncoder(double positionMeters) {
    climberMotor.setPosition(positionMeters);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionMeters, velocityMetersPerSec, statorCurrentAmps, supplyCurrentAmps, voltage, tempC);
    inputs.connected =
        BaseStatusSignal.isAllGood(
            positionMeters,
            velocityMetersPerSec,
            statorCurrentAmps,
            supplyCurrentAmps,
            voltage,
            tempC);
    inputs.positionMeters = positionMeters.getValueAsDouble();
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.velocityMetersPerSec = velocityMetersPerSec.getValueAsDouble();
  }

  @AutoLogOutput(key = "Climber/Setpoint Meters")
  public double getClimberSetpointMeters() {
    return climberSetpoint;
  }
}
