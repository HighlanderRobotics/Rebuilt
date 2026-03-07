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
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import frc.robot.components.camera.Camera.CameraConstants;
import frc.robot.subsystems.swerve.constants.SwerveConstants;
import frc.robot.subsystems.swerve.module.Module.ModuleConstants;

/** Add your docs here. */
public class R1WispSwerveConstants extends SwerveConstants {

  @Override
  public CameraConstants[] getCameraConstants() {
    // TODO all these numbers need to be redone :thumbsup:
    final Matrix<N3, N3> BACK_LEFT_CAMERA_MATRIX =
        MatBuilder.fill(
            Nat.N3(), Nat.N3(), 906.46, 0.0, 675.30, 0.0, 907.49, 394.45, 0.0, 0.0, 1.0);
    final Matrix<N8, N1> BACK_LEFT_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(), Nat.N1(), 0.039, -0.057, -0.005, 0.001, -0.004, -0.001, 0.003, 0.001);
    final Matrix<N3, N3> BACK_RIGHT_CAMERA_MATRIX =
        MatBuilder.fill(
            Nat.N3(), Nat.N3(), 925.82, 0.0, 633.65, 0.0, 927.87, 386.90, 0.0, 0.0, 1.0);
    final Matrix<N8, N1> BACK_RIGHT_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(), Nat.N1(), 0.058, -0.09, 0.006, -0.003, 0.022, -0.002, 0.004, -0.001);
    final Matrix<N3, N3> FRONT_RIGHT_CAMERA_MATRIX =
        MatBuilder.fill(
            Nat.N3(), Nat.N3(), 911.67, 0.0, 663.03, 0.0, 909.82, 408.72, 0.0, 0.0, 1.0);
    final Matrix<N8, N1> FRONT_RIGHT_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(), Nat.N1(), 0.044, -0.069, 0.001, 0.001, 0.013, -0.002, 0.004, 0.001);
    final Matrix<N3, N3> FRONT_LEFT_CAMERA_MATRIX =
        MatBuilder.fill(
            Nat.N3(), Nat.N3(), 920.37, 0.0, 657.16, 0.0, 921.82, 412.98, 0.0, 0.0, 1.0);
    final Matrix<N8, N1> FRONT_LEFT_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(), Nat.N1(), 0.057, -0.09, -0.001, 0.002, 0.043, -0.002, 0.004, -0.002);
    final CameraConstants frontLeftCamConstants =
        new CameraConstants(
            "Front_Left",
            new Transform3d(
                new Translation3d(0.129, 0.339839, 0.201167),
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-20), // -61.875 - 45 + 90),
                    Units.degreesToRadians(60))),
            FRONT_LEFT_CAMERA_MATRIX,
            FRONT_LEFT_DIST_COEFFS);
    final CameraConstants frontRightCamConstants =
        new CameraConstants(
            "Front_Right",
            new Transform3d(
                new Translation3d(0.129788, -0.339178, 0.203421),
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-20), // -61.875 - 45 + 90),
                    Units.degreesToRadians(-60))),
            FRONT_RIGHT_CAMERA_MATRIX,
            FRONT_RIGHT_DIST_COEFFS);
    final CameraConstants backLeftCamConstants =
        new CameraConstants(
            "Back_Left",
            new Transform3d(
                new Translation3d(-0.293657, 0.284027, 0.205814),
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-20), // -61.875 - 45 + 90),
                    Units.degreesToRadians(110))),
            BACK_LEFT_CAMERA_MATRIX,
            BACK_LEFT_DIST_COEFFS);
    final CameraConstants backRightCamConstants =
        new CameraConstants(
            "Back_Right",
            new Transform3d(
                new Translation3d(-0.293657, -0.284027, 0.205814),
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-20), // -61.875 - 45 + 90),
                    Units.degreesToRadians(-110))),
            BACK_RIGHT_CAMERA_MATRIX,
            BACK_RIGHT_DIST_COEFFS);

    return new CameraConstants[] {
      frontLeftCamConstants, frontRightCamConstants, backLeftCamConstants, backRightCamConstants
    };
    // TODO maybe we should standardize on this order
  }

  @Override
  public String getName() {
    return "Wisp";
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
    return Units.inchesToMeters(34.8);
  }

  @Override
  public double getBumperLength() {
    return Units.inchesToMeters(34.8);
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
    // return 9.056;
    return 22.073;
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
    return Pound.of(136.185702);
  }

  // TODO: CANCODER OFFSETS
  @Override
  public ModuleConstants getFrontLeftModuleConstants() {
    return new ModuleConstants(
        0, "Front Left", 0, 1, 0, Rotation2d.fromRotations(-0.22656).plus(Rotation2d.k180deg));
  }

  @Override
  public ModuleConstants getFrontRightModuleConstants() {
    return new ModuleConstants(1, "Front Right", 2, 3, 1, Rotation2d.fromRotations(-0.388));
  }

  @Override
  public ModuleConstants getBackLeftModuleConstants() {
    return new ModuleConstants(
        2, "Back Left", 4, 5, 2, Rotation2d.fromRotations(-0.3).plus(Rotation2d.k180deg));
  }

  @Override
  public ModuleConstants getBackRightModuleConstants() {
    return new ModuleConstants(3, "Back Right", 6, 7, 3, Rotation2d.fromRotations(0.3303));
  }

  @Override
  public int getGyroID() {
    return 0;
  }

  @Override
  public Pigeon2Configuration getGyroConfig() {
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPose.MountPosePitch =
        0.3399254083633423; // -0.20375922322273254; // -0.420589417219162;
    config.MountPose.MountPoseRoll = 179.3086395263672; // -179.13818359375; // -179.8539581298828;
    config.MountPose.MountPoseYaw =
        -90.44647216796875; // -90.93168640136719; // -86.66709899902344;
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
    turnConfig.Slot0.kV = 1.7; // 0.40034; // ((5800 / 60) / getTurnGearRatio()) / 12;
    turnConfig.Slot0.kA = 0.10881; // 0.031543;
    turnConfig.Slot0.kS = 0.7988;
    turnConfig.Slot0.kP = 250.0; // 0; // 50.13; // 20.0;
    turnConfig.Slot0.kD = 1.0; // 3; // 0.68275;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = (7368 / 60) / getTurnGearRatio();
    turnConfig.MotionMagic.MotionMagicAcceleration = (7368 / 60) / (getTurnGearRatio() * 0.005);
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
        // getTurnMotorInverted()
        //     ? SensorDirectionValue.CounterClockwise_Positive
        //     : SensorDirectionValue.Clockwise_Positive;
        SensorDirectionValue.CounterClockwise_Positive;
    return cancoderConfig;
  }

  @Override
  public double getHeadingVelocityKP() {
    // copied from kelpie
    return 6.0;
  }
}
