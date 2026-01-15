// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class HoodIOReal {
  /** Creates a new HoodIOReal. */
  @AutoLog
  public class HoodIOInputs {
    public double hoodPositionMeters = 0.0;
    public double hoodVelocityMetersPerSecond = 0.0;
    public double hoodStatorCurrentAmps = 0.0;
    public double hoodSupplyCurrentAmp = 0.0;
    public double hoodVoltage = 0.0;
    public double hoodTempC = 0.0;
  }

  TalonFX hoodMotor = new TalonFX(1, "*");

  private final BaseStatusSignal hoodPositionMeters = hoodMotor.getPosition();
  private final BaseStatusSignal hoodVelocityMetersPerSec = hoodMotor.getVelocity();
  private final StatusSignal<Voltage> hoodVoltage = hoodMotor.getMotorVoltage();
  private final StatusSignal<Current> hoodStatorCurrent = hoodMotor.getStatorCurrent();
  private final StatusSignal<Current> hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> hoodTemp = hoodMotor.getDeviceTemp();
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);

  public HoodIOReal() {
    
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
    hoodMotor.getConfigurator().apply(config);
        hoodMotor.optimizeBusUtilization();
  }

  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        hoodPositionMeters,
        hoodVelocityMetersPerSec,
        hoodVoltage,
        hoodStatorCurrent,
        hoodSupplyCurrent,
        hoodTemp);

    inputs.hoodPositionMeters = hoodPositionMeters.getValueAsDouble();
    inputs.hoodVelocityMetersPerSecond = hoodVelocityMetersPerSec.getValueAsDouble();
    inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
    inputs.hoodStatorCurrentAmps = hoodStatorCurrent.getValueAsDouble();
    inputs.hoodSupplyCurrentAmp = hoodSupplyCurrent.getValueAsDouble();
    inputs.hoodTempC = hoodTemp.getValueAsDouble();
  }
  
  public void setHoodPosition(Rotation2d hoodPosition) {
    hoodMotor.setControl(positionVoltage.withPosition(hoodPosition.getMeasure()));
  }
}
