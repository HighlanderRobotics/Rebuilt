// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/** Pivoting hooded shooter (turret). !! COMP !! */
public class TurretSubsystem extends SubsystemBase implements Shooter {
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public Command shoot(Supplier<Pose2d> robotPoseSupplier) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'shoot'");
  }

  @Override
  public Command feed(Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> feedTarget) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'feed'");
  }

  @Override
  public Command rest() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'rest'");
  }

  @Override
  public Command spit() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'spit'");
  }

  @Override
  public boolean atFlywheelVelocitySetpoint() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'atFlywheelVelocitySetpoint'");
  }

  @Override
  public boolean atHoodSetpoint() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'atHoodSetpoint'");
  }

  @Override
  public Command zeroHood() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'zeroHood'");
  }

  @Override
  public Command testShoot() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'testShoot'");
  }

  @Override
  public Command runCurrentZeroing() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runCurrentZeroing'");
  }
}
