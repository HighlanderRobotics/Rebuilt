package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIO {

    @AutoLog
    public static class ClimberIOInputs {
        public Rotation2d motorPositionRotations = new Rotation2d();
        public double motorVelocityMetersPerSec = 0.0;
        public double motorStatorCurrentAmps = 0.0;
        public double motorSupplyCurrentAmps = 0.0;
        public double motorVoltage = 0.0;
        public double motorTempC = 0.0;
    }

    protected final TalonFX climberMotor;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> angularVelocityRotsPerSec;
    private final StatusSignal<Current> statorCurrentAmps;
    private final StatusSignal<Current> supplyCurrentAmps;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Temperature> motorTemp;

    private VoltageOut voltageOut = new VoltageOut(0.0) .withEnableFOC(true);
    private PositionVoltage positionVoltage = new PositionVoltage(0.0) .withEnableFOC(true);
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0) .withEnableFOC(true);


public ClimberIO(CANBus canBus) {
    //todo: set correct motor ID
    climberMotor = new TalonFX(30, canBus);
    climberMotor.getConfigurator().apply(ClimberIO.getConfigurator());
        
    angularVelocityRotsPerSec = climberMotor.getVelocity();
    supplyCurrentAmps = climberMotor.getSupplyCurrent();
    motorVoltage = climberMotor.getMotorVoltage();
    statorCurrentAmps = climberMotor.getStatorCurrent();
    motorTemp = climberMotor.getDeviceTemp();
    motorPosition = climberMotor.getPosition();

    //complaining about frequency syntax for some reason
    BaseStatusSignal.setUpdateFrequencyForAll(
        frequencyHz:50.0,
        angularVelocityRotsPerSec,
        supplyCurrentAmps,
        motorVoltage,
        statorCurrentAmps,
        motorTemp,
        motorPosition
        climberMotor.optimizeBusUtilization());
 }

public static TalonFXConfiguration getClimberConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //todo: find and make climber gear ratio variable
    config.Feedback.SensorToMechanismRatio = ClimberSubsystem.GEAR_RATIO;

    //todo: tune 
    config.Slot0.kS = 0.0;
    config.Slot0.KG = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kD = 0.0;

    //todo: find actual current limits
    config.CurrentLimits.StatorCurrentLimit = 50.00;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.00;

    return config;
}

public void setClimberPosition(Rotation2d climberPosition) {
    climberMotor.setControl(positionVoltage.withPosition(climberPosition.getRotations()));
}

public void setClimberVoltage(double climberVoltage) {
    climberMotor.setControl(voltageOut.withOutput(climberVoltage));
}

public void setClimberVelocity(double climberVelocity) {
    climberMotor.setControl(velocityVoltage.withVelocity(climberVelocity));
}

public void updateInputs(ClimberIOInputs inputs) {
  BaseStatusSignal.refreshAll(
    motorPosition,
    angularVelocityRotsPerSec,
    statorCurrentAmps,
    supplyCurrentAmps,
    motorVoltage,
    motorTemp);

  inputs.motorPositionRotations = 
      Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
  inputs.motorVoltage = motorVoltage.getValueAsDouble();
  inputs.motorTempC = motorTempC.getValueAsDouble();
  inputs.motorSupplyCurrentAmps = motorSupplyCurrentAmps.getValueAsDouble();
  inputs.motorStatorCurrentAmps = motorStatorCurrentAmps.getValueAsDouble();
  inputs.motorVelocityMetersPerSec = motorVelocityMetersPerSec.getValueAsDouble();
}
}
  



