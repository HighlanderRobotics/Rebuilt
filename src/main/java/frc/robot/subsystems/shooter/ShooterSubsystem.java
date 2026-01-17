// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.google.common.base.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOInputsAutoLogged;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  HoodIO hoodIO;
  HoodIOInputsAutoLogged hoodinputs = new HoodIOInputsAutoLogged();

  public static double HOOD_GEAR_RATIO = 147.0 / 13.0;
  public static double FLYWHEEL_GEAR_RATIO = 28.0 / 24.0;

  FlywheelIO flywheelIO;
  FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  /** Creates a new HoodSubsystem. */
  public ShooterSubsystem(HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;
  }

  public Command shoot(Supplier<Pose2d> robotPoseSupplier) {
    return this.run(
        () -> {
          ShotData shotData =
              AutoAim.HUB_SHOT_TREE.get(AutoAim.distanceToHub(robotPoseSupplier.get()));
          hoodIO.setHoodPosition(shotData.hoodRotation());
          flywheelIO.setFlywheelVelocity(shotData.flywheelVelocityRotPerSec());
        });
  }

  public Command feed(Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> feedTarget) {
    return this.run(
        () -> {
          ShotData shotData =
              AutoAim.FEED_SHOT_TREE.get(
                  robotPoseSupplier
                      .get()
                      .getTranslation()
                      .getDistance(feedTarget.get().getTranslation()));
          hoodIO.setHoodPosition(shotData.hoodRotation());
          flywheelIO.setFlywheelVelocity(shotData.flywheelVelocityRotPerSec());
        });
  }

  public Command rest() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(Rotation2d.kZero); // TODO: TUNE TUCKED POSITION IF NEEDED
          flywheelIO.setFlywheelVoltage(0.0);
        });
  }

  public Command spit() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(Rotation2d.kZero);
          flywheelIO.setFlywheelVelocity(20);
        }); // TODO: TUNE HOOD POS AND FLYWHEEL VELOCITY
  }

  public Command setHoodPositionCommand(Supplier<Rotation2d> hoodPosition) {
    return this.run(() -> hoodIO.setHoodPosition(hoodPosition.get()));
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodinputs);
    Logger.processInputs("shooter/hood", hoodinputs);

    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("shooter/flywheel", flywheelInputs);
  }
}
