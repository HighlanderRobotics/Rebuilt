// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.constants.comp;

import static edu.wpi.first.units.Units.Pound;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import frc.robot.components.camera.Camera.CameraConstants;
import frc.robot.subsystems.swerve.constants.SwerveConstants;
import frc.robot.subsystems.swerve.module.Module.ModuleConstants;

/** Add your docs here. */
public class R1CompBotSwerveConstants extends SwerveConstants {

  // TODO!!!
  @Override
  public CameraConstants[] getCameraConstants() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCameraConstants'");
  }

  @Override
  public String getName() {
    return "Comp"; // TODO CHANGE ONCE NAMED
  }

  @Override
  public double getTrackWidthX() {
    return Units.inchesToMeters(21.75);
  }

  @Override
  public double getTrackWidthY() {
    return Units.inchesToMeters(21.75);
  }

  @Override
  public double getBumperWidth() {
    return Units.inchesToMeters(34.6);
  }

  @Override
  public double getBumperLength() {
    return Units.inchesToMeters(34.6);
  }

  @Override
  public double getMaxLinearSpeed() {
    // From https://www.swervedrivespecialties.com/collections/kits/products/mk5n-swerve-module
    // SDS Mk5n, R1 ratio, no FOC (because FOC is disabled if we're going fast enough)
    return Units.feetToMeters(14.9);
  }

  @Override
  public double getMaxLinearAcceleration() {
    // Calculated in Choreo for R1 ratio
    return 9.056;
  }

  @Override
  public double getDriveGearRatio() {
    // From https://www.swervedrivespecialties.com/collections/kits/products/mk5n-swerve-module
    // Mk5n, R1 ratio
    return 7.03;
  }

  @Override
  public double getTurnGearRatio() {
    // From https://www.swervedrivespecialties.com/collections/kits/products/mk5n-swerve-module
    return 287 / 11;
  }

  @Override
  public Mass getMass() {
    // From CAD (retrieved 1/29/26), with bumpers and battery
    return Pound.of(121.28);
  }

  // TODO: CANCODER OFFSETS
  @Override
  public ModuleConstants getFrontLeftModuleConstants() {
    return new ModuleConstants(0, "Front Left", 0, 1, 0, Rotation2d.fromRotations(0.0));
  }

  @Override
  public ModuleConstants getFrontRightModuleConstants() {
    return new ModuleConstants(1, "Front Right", 2, 3, 1, Rotation2d.fromRotations(0.0));
  }

  @Override
  public ModuleConstants getBackLeftModuleConstants() {
    return new ModuleConstants(2, "Back Left", 4, 5, 2, Rotation2d.fromRotations(0.0));
  }

  @Override
  public ModuleConstants getBackRightModuleConstants() {
    return new ModuleConstants(3, "Back Right", 6, 7, 3, Rotation2d.fromRotations(0.0));
  }

  @Override
  public int getGyroID() {
    return 0;
  }

  @Override
  public Pigeon2Configuration getGyroConfig() {
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPose.MountPosePitch = 0.0;
    config.MountPose.MountPoseRoll = 0.0;
    config.MountPose.MountPoseYaw = 0.0;
    return config;
  }

  @Override
  public TalonFXConfiguration getDriveConfig() {
    var driveConfig = new TalonFXConfiguration();
    // Current limits
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Inverts
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Sensor
    // Meters per second
    driveConfig.Feedback.SensorToMechanismRatio = getDriveRotorToMeters();
    // Current control gains
    // Gains copied from Kelpie Swerve Constants
    // May need tuning
    driveConfig.Slot0.kV = 5.0;
    // kT (stall torque / stall current) converted to linear wheel frame
    driveConfig.Slot0.kA = 0.0; // (9.37 / 483.0) / getDriveRotorToMeters(); // 3.07135116146;
    driveConfig.Slot0.kS = 10.0;
    driveConfig.Slot0.kP = 300.0;
    driveConfig.Slot0.kD = 0.0; // 1.0;

    driveConfig.TorqueCurrent.TorqueNeutralDeadband = 10.0;

    driveConfig.MotionMagic.MotionMagicCruiseVelocity = getMaxLinearSpeed();
    driveConfig.MotionMagic.MotionMagicAcceleration = getMaxLinearAcceleration();

    return driveConfig;
  }

  @Override
  public TalonFXConfiguration getTurnConfig(int cancoderID) {
    var turnConfig = new TalonFXConfiguration();
    // Current limits
    turnConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Inverts
    turnConfig.MotorOutput.Inverted =
        getTurnMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Fused Cancoder
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
    turnConfig.Feedback.RotorToSensorRatio = getTurnGearRatio();
    turnConfig.Feedback.SensorToMechanismRatio = 1.0;
    turnConfig.Feedback.FeedbackRotorOffset = 0.0;
    // Controls Gains
    // Copied from Kelpie
    turnConfig.Slot0.kV = ((5800 / 60) / getTurnGearRatio()) / 12;
    turnConfig.Slot0.kA = 0.031543;
    turnConfig.Slot0.kS = 0.27;
    turnConfig.Slot0.kP = 20.0;
    turnConfig.Slot0.kD = 0.68275;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = (5500 / 60) / getTurnGearRatio();
    turnConfig.MotionMagic.MotionMagicAcceleration = (5500 / 60) / (getTurnGearRatio() * 0.005);
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    return turnConfig;
  }

  @Override
  public boolean getTurnMotorInverted() {
    return false; // Checked this on a module
  }

  @Override
  public MotorType getTurnMotorType() {
    return MotorType.KrakenX44;
  }

  @Override
  public MotorType getDriveMotorType() {
    return MotorType.KrakenX60;
  }

  @Override
  public CANcoderConfiguration getCancoderConfig(Rotation2d cancoderOffset) {
    final var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = cancoderOffset.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        getTurnMotorInverted()
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    return cancoderConfig;
  }

  @Override
  public double getHeadingVelocityKP() {
    // copied from kelpie
    return 6.0;
  }
}
