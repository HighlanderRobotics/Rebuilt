// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Pivoting hooded shooter (turret). !! COMP !! */
public class TurretSubsystem extends SubsystemBase implements Shooter {
  public TurretIO turretIO;
  public TurretIOInputsAutoLogged turretIOInputs = new TurretIOInputsAutoLogged();

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(TurretIO turretIO) {
    this.turretIO = turretIO;
  }

  @Override
  public void periodic() {
    turretIO.updateInputs(turretIOInputs);
    Logger.processInputs("Shooter/Turret", turretIOInputs);
    // This method will be called once per scheduler run
  }

  @Override
  public Command shoot(Supplier<Pose2d> robotPoseSupplier) {
    return this.run(
        () -> {
          turretIO.setTurretPosition(new Rotation2d());
          // TODO:Find the reall number

        });
  }

  @Override
  public Command feed(Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> feedTarget) {
    return this.run(
        () -> {
          turretIO.setTurretPosition(new Rotation2d());
          // TODO:Find the reall number

        });
  }

  @Override
  public Command rest() {
    return this.run(
        () -> {
          turretIO.setTurretPosition(new Rotation2d());
          // TODO:Find the reall number

        });
  }

  @Override
  public Command spit() {
    return this.run(
        () -> {
          turretIO.setTurretPosition(new Rotation2d());
          // TODO:Find the reall number

        });
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
    return this.run(
        () -> {
          turretIO.setTurretPosition(new Rotation2d());
          // TODO:Find the reall number

        });
  }
}
