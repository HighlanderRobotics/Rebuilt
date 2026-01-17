// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public class FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public double flywheelPositionMeters = 0.0;
    public double flywheelVelocityMetersPerSecond = 0.0;
    public double flywheelStatorCurrentAmps = 0.0;
    public double flywheelSupplyCurrentAmp = 0.0;
    public double flywheelVoltage = 0.0;
    public double flywheelTempC = 0.0;
  }

  protected TalonFX flywheelLeader;
  protected TalonFX flywheelFollower;

  private final BaseStatusSignal flywheelPositionRotations;
  private final BaseStatusSignal flywheelVelocity;
  private final StatusSignal<Voltage> flywheelVoltage;
  private final StatusSignal<Current> flywheelStatorCurrent;
  private final StatusSignal<Current> flywheelSupplyCurrent;
  private final StatusSignal<Temperature> flywheelTemp;
  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

  public FlywheelIO(TalonFXConfiguration config, CANBus canbus) {
    flywheelLeader = new TalonFX(10, canbus);
    flywheelFollower = new TalonFX(11, canbus);

    flywheelLeader.getConfigurator().apply(config);
    flywheelFollower.getConfigurator().apply(config);

    // follower follows leader
    flywheelFollower.setControl(
        new Follower(
            flywheelLeader.getDeviceID(),
            MotorAlignmentValue.Opposed) // i didnt know what to put here
        );

    flywheelVelocity = flywheelLeader.getVelocity();
    flywheelVoltage = flywheelLeader.getMotorVoltage();
    flywheelStatorCurrent = flywheelLeader.getStatorCurrent();
    flywheelSupplyCurrent = flywheelLeader.getSupplyCurrent();
    flywheelTemp = flywheelLeader.getDeviceTemp();
    flywheelPositionRotations = flywheelLeader.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelLeader.getVelocity(),
        flywheelLeader.getMotorVoltage(),
        flywheelLeader.getStatorCurrent(),
        flywheelLeader.getSupplyCurrent(),
        flywheelLeader.getDeviceTemp());

    flywheelLeader.optimizeBusUtilization();
    flywheelFollower.optimizeBusUtilization();
  }

  // did not know how to do anything here,
  public static TalonFXConfiguration getFlywheelConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kS = 0.2;
    config.Slot0.kV = 0.12;
    config.Slot0.kP = 0.3;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 80.0;

    return config;
  }

  public void setFlywheelVoltage(double volts) {
    flywheelLeader.setControl(voltageOut.withOutput(volts));
  }

  public void setFlywheelVelocity(double flywheelVelocity) {
    flywheelLeader.setControl(velocityVoltage.withVelocity(flywheelVelocity));
  }

  public void stop() { // thought i should add a stop command, dont think i had to though
    flywheelLeader.setControl(voltageOut.withOutput(0.0));
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        flywheelVelocity,
        flywheelVoltage,
        flywheelStatorCurrent,
        flywheelSupplyCurrent,
        flywheelTemp);

    inputs.flywheelVelocityMetersPerSecond = flywheelVelocity.getValueAsDouble();
    inputs.flywheelVoltage = flywheelVoltage.getValueAsDouble();
    inputs.flywheelStatorCurrentAmps = flywheelStatorCurrent.getValueAsDouble();
    inputs.flywheelSupplyCurrentAmp = flywheelSupplyCurrent.getValueAsDouble();
    inputs.flywheelTempC = flywheelTemp.getValueAsDouble();
  }
}
