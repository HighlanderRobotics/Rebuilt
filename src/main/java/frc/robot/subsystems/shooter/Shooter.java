// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/** Add your docs here. */
public interface Shooter {

  public Command shoot(Supplier<Pose2d> robotPoseSupplier);

  public Command feed(Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> feedTarget);

  public Command rest();

  public Command spit();

  public boolean atFlywheelVelocitySetpoint();

  public boolean atHoodSetpoint();

  public Command zeroHood();

  public Command testShoot();
}
