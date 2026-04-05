// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public interface Indexer extends Subsystem {

  /** Run indexer towards shooter and kicker away from shooter */
  public Command index();

  /** Run everything backwards. This is for antijamming the robot */
  public Command spit();

  /** Run both indexer and kicker towards the shooter */
  public Command kick();

  /** Not running (set spinner to 0 but idle kicker) */
  public Command rest();

  public default Command stop() {
    return Commands.none();
  }

  public default Command runX60Sysid() {
    return Commands.none();
  }

  public default Command runX44Sysid() {
    return Commands.none();
  }
}
