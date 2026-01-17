// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.autoaim.AutoAim;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
  private HoodIO hoodIO;
  private HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  public static double GEAR_RATIO = 147.0 / 13.0;

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem(HoodIO io) {
    this.hoodIO = io;
  }

  public Command setHoodPositionCommand(Supplier<Rotation2d> hoodPosition) {
    return this.run(() -> hoodIO.setHoodPosition(hoodPosition.get()));
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
  }

  public Command shoot(Supplier<Pose2d> robotPoseSupplier) {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(
              AutoAim.HUB_SHOT_TREE
                  .get(AutoAim.distanceToHub(robotPoseSupplier.get()))
                  .hoodRotation());
          // TODO: FLYWHEEL WHEN MERGED
        });
  }

  public Command feed(Supplier<Pose2d> robotPoseSupplier) {
    return shoot(robotPoseSupplier); // TODO
  }

  public Command rest() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(Rotation2d.kZero); // TODO: TUNE TUCKED POSITION IF NEEDED
          // TODO: FLYWHEEL
        });
  }

  public Command spit() {
    return this.run(
        () -> hoodIO.setHoodPosition(Rotation2d.kZero)); // TODO: FLYWHEEL AND TUNE HOOD POS
  }
}
