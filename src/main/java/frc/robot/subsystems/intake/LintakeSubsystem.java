// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Lintake = Linear Intake. !! COMP !! */
public class LintakeSubsystem extends SubsystemBase implements Intake {
  /** Creates a new LintakeSubsystem. */
  public LintakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public Command intake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'intake'");
  }

  @Override
  public Command outtake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'outtake'");
  }

  @Override
  public Command rest() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'rest'");
  }

  @Override
  public void close() throws Exception {
      // No-op rn bc nothing to close
  }
}
