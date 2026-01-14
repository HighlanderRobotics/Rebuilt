// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
  HoodIOReal io;
  HoodIOInputsAutologged inputs = new HoodIOInputsAutologged();

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem(HoodIOReal io) {
    this.io = io;
  }

  private void setHoodVoltage(double hoodVoltage) {
    io.setHoodVoltage(hoodVoltage);
  }

  private void setHoodPosition(Rotation2d hoodPosition) {
    io.setHoodPosition(hoodPosition);
  }

  private void setHoodVelocity(double hoodVelocity) {
    io.setHoodVelocity(hoodVelocity);
  }

  public Command setHoodVoltageCommand(Double hoodVoltage) {
    return this.run(() -> this.setHoodVoltage(hoodVoltage));
  }

  public Command setHoodPositionCommand(Rotation2d hoodPosition) {
    return this.run(() -> this.setHoodPosition(hoodPosition));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("hood", inputs);
  }
}
