// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Spindexer = SPINning Indexer. !! COMP !! */
public class SpindexerSubsystem extends SubsystemBase implements Indexer {
  /** Creates a new SpindexerSubsystem. */
  public SpindexerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public boolean isFull() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'isFull'");
  }

  @Override
  public boolean isEmpty() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'isEmpty'");
  }

  @Override
  public boolean isPartiallyFull() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'isPartiallyFull'");
  }

  @Override
  public Command index() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'index'");
  }

  @Override
  public Command shoot() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'shoot'");
  }

  @Override
  public Command spit() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'spit'");
  }

  @Override
  public Command kick() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'kick'");
  }

  @Override
  public Command rest() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'rest'");
  }
}
