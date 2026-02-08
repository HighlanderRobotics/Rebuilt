// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public interface Intake {
  /** Run balls towards the shooter */
  public Command intake();

  /** Run balls away from the shooter. This is for antijamming the robot */
  public Command outtake();

  /** Not running (set to 0) */
  public Command rest();

  public Command extend();
}
