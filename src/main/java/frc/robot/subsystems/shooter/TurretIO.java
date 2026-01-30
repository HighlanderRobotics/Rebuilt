// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public Rotation2d turretPositionRotations = new Rotation2d();
        public double turretStatorCurrentAmps = 0.0;
        public double turretSupplyCurrentAmps = 0.0;
        public double turretVoltage = 0.0;
        public double turretTempC = 0.0;
        //_TODO: Input reall values 
    }
    protected TalonFX turretMotor;
    private final BaseStatusSignal turretPositionRotations;
     private final StatusSignal<Voltage> turretVoltage;
     private final StatusSignal<Current> turretStatorCurrent;
     private final StatusSignal<Current> turretSupplyCurrent;
    private final StatusSignal<Temperature> turretTemp;
    private final StatusSignal<AngularVelocity> turretAngularVelocity;
    private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

   public TurretIO(TalonFXConfiguration talonFXConfiguration, CANBus canbus) {
    turretMotor = new TalonFX(11, canbus);
    turretMotor.getConfigurator().apply(TurretIO.getTurretConfiguration());
        turretPositionRotations = turretMotor.getPosition();
        turretAngularVelocity = turretMotor.getVelocity();
        turretVoltage = turretMotor.getMotorVoltage();
        turretStatorCurrent = turretMotor.getStatorCurrent();
        turretSupplyCurrent = turretMotor.getSupplyCurrent();
        turretTemp = turretMotor.getDeviceTemp();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            turretPositionRotations,
            turretAngularVelocity,
            turretVoltage,
            turretStatorCurrent,
            turretSupplyCurrent,
            turretTemp);
        turretMotor.optimizeBusUtilization();
      }
    
      
    
    public static TalonFXConfiguration getTurretConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ShooterSubsystem.TURRET_GEAR_RATIO;

    // config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; Potentially need, maybe not tho.

    config.Slot0.kS = 0.0;
    config.Slot0.kG = 0.0;
    config.Slot0.kV = 1.1;
    config.Slot0.kP = 5.0;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;

    return config;
}
 public void setTurretVoltage(double turretVoltage) {
    turretMotor.setControl(voltageOut.withOutput(turretVoltage));
  }

  public void setturretPosition(Rotation2d turretPosition) {
    turretMotor.setControl(positionVoltage.withPosition(turretPosition.getRotations()));
  }

  public void setTurretVelocity(double turretVelocity) {
    turretMotor.setControl(velocityVoltage.withVelocity(turretVelocity));
  }

  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        turretPositionRotations,
        turretVoltage,
        turretStatorCurrent,
        turretSupplyCurrent,
        turretTemp);

    inputs.turretPositionRotations = Rotation2d.fromRadians(turretPositionRotations.getValueAsDouble());
    inputs.turretVoltage = turretVoltage.getValueAsDouble();
    inputs.turretStatorCurrentAmps = turretStatorCurrent.getValueAsDouble();
    inputs.turretSupplyCurrentAmps = turretSupplyCurrent.getValueAsDouble();
    inputs.turretTempC = turretTemp.getValueAsDouble();
  }
}
