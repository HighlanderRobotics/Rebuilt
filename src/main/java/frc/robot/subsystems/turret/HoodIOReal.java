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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class HoodIOReal {
  /** Creates a new HoodIOReal. */
  @AutoLog
  public static class HoodIOInputs {
    public Rotation2d hoodPositionRotations = Rotation2d.kZero;
    public double hoodAngularVelocityRotsPerSec = 0.0;
    public double hoodStatorCurrentAmps = 0.0;
    public double hoodSupplyCurrentAmp = 0.0;
    public double hoodVoltage = 0.0;
    public double hoodTempC = 0.0;
  }

  TalonFX hoodMotor = new TalonFX(10, "*");

  private final StatusSignal<Angle> hoodPositionRotations = hoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> hoodVelocityRotsPerSec = hoodMotor.getVelocity();
  private final StatusSignal<Voltage> hoodVoltage = hoodMotor.getMotorVoltage();
  private final StatusSignal<Current> hoodStatorCurrent = hoodMotor.getStatorCurrent();
  private final StatusSignal<Current> hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> hoodTemp = hoodMotor.getDeviceTemp();
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);

  public HoodIOReal() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //took from 254 idec
    config.Slot0.kS = 0.18;
    config.Slot0.kP = 8.0;
    config.Slot0.kD = 0.1;
    config.Slot0.kV = 0.116;
    config.Slot0.kA = 0.0001 * 12.0;

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
        hoodPositionRotations,
        hoodVelocityRotsPerSec,
        hoodVoltage,
        hoodStatorCurrent,
        hoodSupplyCurrent,
        hoodTemp);

    inputs.hoodPositionRotations = Rotation2d.fromRotations(hoodPositionRotations.getValueAsDouble());
    inputs.hoodAngularVelocityRotsPerSec = hoodVelocityRotsPerSec.getValueAsDouble();
    inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
    inputs.hoodStatorCurrentAmps = hoodStatorCurrent.getValueAsDouble();
    inputs.hoodSupplyCurrentAmp = hoodSupplyCurrent.getValueAsDouble();
    inputs.hoodTempC = hoodTemp.getValueAsDouble();
  }

  public void setHoodPosition(Rotation2d hoodPosition) {
    hoodMotor.setControl(positionVoltage.withPosition(hoodPosition.getMeasure()));
  }
}
