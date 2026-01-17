// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

// idk if this is what i was supposed to import

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.AutoLog;

public class HoodIO {
  /** Creates a new HoodIOReal. */
  @AutoLog
  public static class HoodIOInputs {
    public Rotation2d hoodPositionRotations = new Rotation2d();
    public double hoodAngularVelocity = 0.0;
    public double hoodStatorCurrentAmps = 0.0;
    public double hoodSupplyCurrentAmp = 0.0;
    public double hoodVoltage = 0.0;
    public double hoodTempC = 0.0;
  }

  protected TalonFX hoodMotor;

  private final BaseStatusSignal hoodPositionRotations;
  private final BaseStatusSignal hoodAngularVelocity;
  private final StatusSignal<Voltage> hoodVoltage;
  private final StatusSignal<Current> hoodStatorCurrent;
  private final StatusSignal<Current> hoodSupplyCurrent;
  private final StatusSignal<Temperature> hoodTemp;
  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

  public HoodIO(TalonFXConfiguration talonFXConfiguration, CANBus canbus) {
    hoodMotor = new TalonFX(11, canbus);
    hoodMotor.getConfigurator().apply(HoodIO.getHoodConfiguration());

    hoodPositionRotations = hoodMotor.getPosition();
    hoodAngularVelocity = hoodMotor.getVelocity();
    hoodVoltage = hoodMotor.getMotorVoltage();
    hoodStatorCurrent = hoodMotor.getStatorCurrent();
    hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
    hoodTemp = hoodMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        hoodPositionRotations,
        hoodAngularVelocity,
        hoodVoltage,
        hoodStatorCurrent,
        hoodSupplyCurrent,
        hoodTemp);
    hoodMotor.optimizeBusUtilization();
  }

  public static TalonFXConfiguration getHoodConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ShooterSubsystem.HOOD_GEAR_RATIO;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

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

  public void setHoodVoltage(double hoodVoltage) {
    hoodMotor.setControl(voltageOut.withOutput(hoodVoltage));
  }

  public void setHoodPosition(Rotation2d hoodPosition) {
    hoodMotor.setControl(positionVoltage.withPosition(hoodPosition.getRotations()));
  }

  public void setHoodVelocity(double hoodVelocity) {
    hoodMotor.setControl(velocityVoltage.withVelocity(hoodVelocity));
  }

  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        hoodPositionRotations,
        hoodAngularVelocity,
        hoodVoltage,
        hoodStatorCurrent,
        hoodSupplyCurrent,
        hoodTemp);

    inputs.hoodPositionRotations = Rotation2d.fromRadians(hoodPositionRotations.getValueAsDouble());
    inputs.hoodAngularVelocity = hoodAngularVelocity.getValueAsDouble();
    inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
    inputs.hoodStatorCurrentAmps = hoodStatorCurrent.getValueAsDouble();
    inputs.hoodSupplyCurrentAmp = hoodSupplyCurrent.getValueAsDouble();
    inputs.hoodTempC = hoodTemp.getValueAsDouble();
  }
}
