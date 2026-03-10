// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import java.util.function.Supplier;

/** Add your docs here. */
public interface Shooter extends Subsystem {

  /**
   * Sets hood angle and flywheel velocity based on distance from hub from the shot map + current
   * pose
   */
  public Command score(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ShotData> shotDataSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier);

  /**
   * Sets hood angle and flywheel velocity based on distance from hub from the feed map + current
   * pose + feed target
   */
  public Command feed(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ShotData> shotDataSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<Pose2d> feedTarget);

  /** Not running (set to 0) */
  public Command rest(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier);

  /** Run balls out from the shooter. This is for antijamming the robot */
  public Command spit();

  /** Check if flywheel has spun up to desired speed */
  public boolean atFlywheelVelocitySetpoint();

  /** Check if hood is at its desired position */
  public boolean atHoodSetpoint();

  /** Reset hood encoder to its minimum position */
  public Command zeroHood();

  /** Shoots based on dashboard numbers. For testing only */
  public Command testShoot(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier);

  public default Command torqueCurrentTest(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    return Commands.none();
  }

  /**
   * Runs the hood backwards until it hits its hard stop and the current spikes, then resets encoder
   * position.
   */
  public Command runCurrentZeroing();

  public Rotation2d getHoodSetpoint();

  public Rotation2d getHoodPosition();

  public boolean atTurretSetpoint();

  public Command runFlywheelSysid();

  public Command runHoodSysid();

  public default Command runTurretSysid() {
    return Commands.none();
  }

  public default Command resetTurretToPosition(Rotation2d rot) {
    return Commands.none();
  }

  public default Rotation2d getCalculatedTurretRotations() {
    return Rotation2d.kZero;
  }

  public default Rotation2d getTurretPosition() {
    return Rotation2d.kZero;
  }

  public default Rotation2d getTurretSetpoint() {
    return Rotation2d.kZero;
  }

  public default Command spinUpTest(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    return Commands.none();
  }

  public default Command spinUp(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ShotData> shotDataSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    return Commands.none();
  }

  public default void turretInit() {}

  public default Pose2d getTurretPose(Pose2d robotPose) {
    return Pose2d.kZero;
  }

    public default Command currentZeroTurret() {
      return Commands.none();
    }
}
