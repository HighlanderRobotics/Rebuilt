// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Pivoting hooded shooter (turret). !! COMP !! */
public class TurretSubsystem extends SubsystemBase implements Shooter {
  /** Creates a new TurretSubsystem. */
  public static double HOOD_GEAR_RATIO_C = 58.96875;
  public static double FLYWHEEL_GEAR_RATIO_C = 0.84615384615;

  public static Rotation2d HOOD_MAX_ROTATION = Rotation2d.fromDegrees(40);
  public static Rotation2d HOOD_MIN_ROTATION = Rotation2d.fromDegrees(2);

  

  public static double FLYWHEEL_VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND = 5.0;

  HoodIO hoodIO;
  HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  FlywheelIO flywheelIO;
  FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  public TurretSubsystem(FlywheelIO flywheelIO, HoodIO hoodIO) {
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
  }

  private LoggedTunableNumber testDegrees = new LoggedTunableNumber("Shooter/Test Degrees", 10.0);
  private LoggedTunableNumber testVelocity = new LoggedTunableNumber("Shooter/Test Velocity", 30.0);

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
    
  }

  @Override
  public Command feed(Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> feedTarget) {
    return this.run(
        () -> {
          ShotData shotData =
              AutoAim.FEED_SHOT_TREE.get(
                  robotPoseSupplier
                      .get()
                      .getTranslation()
                      .getDistance(feedTarget.get().getTranslation()));
          hoodIO.setHoodPosition(shotData.hoodAngle());
          flywheelIO.setMotionProfiledFlywheelVelocity(shotData.flywheelVelocityRotPerSec());
        });
  }

  @Override
  public Command rest() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(HOOD_MIN_ROTATION); // TODO: TUNE TUCKED POSITION IF NEEDED
          flywheelIO.setFlywheelVoltage(0.0);
        });
  }

  @Override
  public Command spit() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(Rotation2d.kZero);
          flywheelIO.setMotionProfiledFlywheelVelocity(20);
        }); // TODO: TUNE HOOD POS AND FLYWHEEL VELOCITY
  }

  @Override
  @AutoLogOutput(key = "Shooter/At Vel Setpoint")
  public boolean atFlywheelVelocitySetpoint() {
    return MathUtil.isNear(
        flywheelInputs.flywheelLeaderVelocityRotationsPerSecond,
        flywheelIO.getSetpointRotPerSec(),
        FLYWHEEL_VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND);
  }

  @Override
  @AutoLogOutput(key = "Shooter/Hood/At Setpoint")
  public boolean atHoodSetpoint() {
    return MathUtil.isNear(
        hoodInputs.hoodPositionRotations.getDegrees(), hoodIO.getHoodSetpoint().getDegrees(), 1);
  }

  @Override
  public Command zeroHood() {
    return this.runOnce(() -> hoodIO.resetEncoder(HOOD_MIN_ROTATION));
  }

  @Override
  public Command testShoot() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(Rotation2d.fromDegrees(testDegrees.get()));
          flywheelIO.setMotionProfiledFlywheelVelocity(testVelocity.get());
        });
  }

  @Override
  public Command shoot(Supplier<Pose2d> robotPoseSupplier) {
    return this.run(
        () -> {
          ShotData shotData =
              AutoAim.HUB_SHOT_TREE.get(AutoAim.distanceToHub(robotPoseSupplier.get()));
          hoodIO.setHoodPosition(shotData.hoodAngle());
          flywheelIO.setMotionProfiledFlywheelVelocity(shotData.flywheelVelocityRotPerSec());
        });
  }
}
