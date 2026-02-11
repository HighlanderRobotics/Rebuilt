// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public interface Indexer {

  /** Run indexer towards shooter and kicker away from shooter */
  public Command index();

  /** Run everything backwards. This is for antijamming the robot */
  public Command spit();

  /** Run both indexer and kicker towards the shooter */
  public Command kick();

  /** Not running (set to 0) */
  public Command rest();

  public Command runRollerSysId();

  public default Command runKickerSysId() {
    return Commands.none();
  } // alpha also has a kicker but ig we didn't use sysid for that and i'm too lazy to add it just
  // for this
}
