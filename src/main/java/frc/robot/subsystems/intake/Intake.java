// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public interface Intake extends Subsystem {
  /** Run balls towards the shooter */
  public Command intake();

  /** Run balls away from the shooter. This is for antijamming the robot */
  public Command agitate();

  /** Not running (set to 0) */
  public Command restExtended();

  /** for controller rumble */
  public boolean beambreak();

  public Rotation2d getPosition();

  public Rotation2d getPositionSetpoint();

  public Command runRollerSysid();

  public default Command runPivotSysid() {
    return Commands.none();
  }

  public Command zeroPivotOffCancoder();

  public Command runCurrentZeroing();

  public default Command climb() {
    return Commands.none();
  }

  public default Command restRetracted() {
    return Commands.none();
  }
}
