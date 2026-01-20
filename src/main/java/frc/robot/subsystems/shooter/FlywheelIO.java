// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public class FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public double flywheelLeaderVelocityRotationsPerSecond = 0.0;
    public double flywheelLeaderStatorCurrentAmps = 0.0;
    public double flywheelLeaderSupplyCurrentAmp = 0.0;
    public double flywheelLeaderVoltage = 0.0;
    public double flywheelLeaderTempC = 0.0;
    public double flywheelLeaderPosition = 0.0;

    public double flywheelFollowerVelocityRotationsPerSecond = 0.0;
    public double flywheelFollowerStatorCurrentAmps = 0.0;
    public double flywheelFollowerSupplyCurrentAmp = 0.0;
    public double flywheelFollowerVoltage = 0.0;
    public double flywheelFollowerTempC = 0.0;
  }

  protected TalonFX flywheelLeader;
  protected TalonFX flywheelFollower;

  private final BaseStatusSignal flywheelLeaderVelocity;
  private final StatusSignal<Voltage> flywheelLeaderVoltage;
  private final StatusSignal<Current> flywheelLeaderStatorCurrent;
  private final StatusSignal<Current> flywheelLeaderSupplyCurrent;
  private final StatusSignal<Temperature> flywheelLeaderTemp;
  private final StatusSignal<Angle> flywheelLeaderPosition;

  private final BaseStatusSignal flywheelFollowerVelocity;
  private final StatusSignal<Voltage> flywheelFollowerVoltage;
  private final StatusSignal<Current> flywheelFollowerStatorCurrent;
  private final StatusSignal<Current> flywheelFollowerSupplyCurrent;
  private final StatusSignal<Temperature> flywheelFollowerTemp;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);
  private MotionMagicVelocityVoltage motionMagicVelocityVoltage;

  private double velocitySetpointRotPerSec = 0.0;

  // todo: tune acceleration

  public FlywheelIO(TalonFXConfiguration config, CANBus canbus) {
    flywheelLeader = new TalonFX(12, canbus);
    flywheelFollower = new TalonFX(13, canbus);

    flywheelLeader.getConfigurator().apply(config);
    flywheelFollower.getConfigurator().apply(config);

    // follower follows leader
    flywheelFollower.setControl(
        new Follower(flywheelLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    flywheelLeaderVelocity = flywheelLeader.getVelocity();
    flywheelLeaderVoltage = flywheelLeader.getMotorVoltage();
    flywheelLeaderStatorCurrent = flywheelLeader.getStatorCurrent();
    flywheelLeaderSupplyCurrent = flywheelLeader.getSupplyCurrent();
    flywheelLeaderTemp = flywheelLeader.getDeviceTemp();
    flywheelLeaderPosition = flywheelLeader.getPosition();

    flywheelFollowerVelocity = flywheelFollower.getVelocity();
    flywheelFollowerVoltage = flywheelFollower.getMotorVoltage();
    flywheelFollowerStatorCurrent = flywheelFollower.getStatorCurrent();
    flywheelFollowerSupplyCurrent = flywheelFollower.getSupplyCurrent();
    flywheelFollowerTemp = flywheelFollower.getDeviceTemp();

    motionMagicVelocityVoltage =
        new MotionMagicVelocityVoltage(
                0.0).withAcceleration(
                config.MotionMagic.MotionMagicAcceleration)
            .withEnableFOC(true);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelLeader.getVelocity(),
        flywheelLeader.getMotorVoltage(),
        flywheelLeader.getStatorCurrent(),
        flywheelLeader.getSupplyCurrent(),
        flywheelLeader.getDeviceTemp(),
        flywheelFollower.getVelocity(),
        flywheelFollower.getMotorVoltage(),
        flywheelFollower.getStatorCurrent(),
        flywheelFollower.getSupplyCurrent(),
        flywheelFollower.getDeviceTemp(),
        flywheelLeaderPosition);

    flywheelLeader.optimizeBusUtilization();
    flywheelFollower.optimizeBusUtilization();
  }

  public static TalonFXConfiguration getFlywheelConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ShooterSubsystem.FLYWHEEL_GEAR_RATIO;

    config.Slot0.kS = 0.43477;
    config.Slot0.kV = 0.144; // From sim
    config.Slot0.kA = 0.016433;
    config.Slot0.kP = 0.1;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 80.0;
 
    config.MotionMagic.MotionMagicAcceleration = 100.0;

    return config;
  }

  public void setFlywheelVoltage(double volts) {
    flywheelLeader.setControl(voltageOut.withOutput(volts));
  }

  public void setMotionProfiledFlywheelVelocity(double flywheelVelocity) {
    velocitySetpointRotPerSec = flywheelVelocity;
    flywheelLeader.setControl(motionMagicVelocityVoltage.withVelocity(flywheelVelocity));
  }

  public void stop() { // thought i should add a stop command, dont think i had to though
    velocitySetpointRotPerSec = 0.0;
    flywheelLeader.setControl(voltageOut.withOutput(0.0));
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        flywheelLeaderVelocity,
        flywheelLeaderVoltage,
        flywheelLeaderStatorCurrent,
        flywheelLeaderSupplyCurrent,
        flywheelLeaderTemp,
        flywheelFollowerVelocity,
        flywheelFollowerVoltage,
        flywheelFollowerStatorCurrent,
        flywheelFollowerSupplyCurrent,
        flywheelFollowerTemp,
        flywheelLeaderPosition);

    inputs.flywheelLeaderVelocityRotationsPerSecond = flywheelLeaderVelocity.getValueAsDouble();
    inputs.flywheelLeaderVoltage = flywheelLeaderVoltage.getValueAsDouble();
    inputs.flywheelLeaderStatorCurrentAmps = flywheelLeaderStatorCurrent.getValueAsDouble();
    inputs.flywheelLeaderSupplyCurrentAmp = flywheelLeaderSupplyCurrent.getValueAsDouble();
    inputs.flywheelLeaderTempC = flywheelLeaderTemp.getValueAsDouble();

    inputs.flywheelLeaderPosition = flywheelLeaderPosition.getValueAsDouble();

    inputs.flywheelFollowerVelocityRotationsPerSecond = flywheelFollowerVelocity.getValueAsDouble();
    inputs.flywheelFollowerVoltage = flywheelFollowerVoltage.getValueAsDouble();
    inputs.flywheelFollowerStatorCurrentAmps = flywheelFollowerStatorCurrent.getValueAsDouble();
    inputs.flywheelFollowerSupplyCurrentAmp = flywheelFollowerSupplyCurrent.getValueAsDouble();
    inputs.flywheelFollowerTempC = flywheelFollowerTemp.getValueAsDouble();
  }

  public double getSetpointRotPerSec() {
    return velocitySetpointRotPerSec;
  }
}
