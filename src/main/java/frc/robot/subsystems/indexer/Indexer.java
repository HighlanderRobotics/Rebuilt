// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public interface Indexer {

  public boolean isFull();

  public boolean isEmpty();

  public boolean isPartiallyFull();

  public Command index();

  public Command shoot();

  public Command spit();

  public Command kick();

  public Command rest();
}
