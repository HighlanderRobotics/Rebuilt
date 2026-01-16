// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import frc.robot.components.rollers.RollerIOReal;

public class HoodSubsystem extends SubsystemBase {
  HoodIOReal hoodIO;
  HoodIOInputsAutoLogged hoodinputs = new HoodIOInputsAutoLogged();

  RollerIOReal flywheelIO;
  RollerIOInputsAutoLogged flywheelInputs = new RollerIOInputsAutoLogged();

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem(HoodIOReal hoodIO, RollerIOReal rollerIO) {
    this.hoodIO = hoodIO;
    this.flywheelIO = rollerIO;
  }

  public Command shoot(DoubleSupplier voltage) {
    return this.run(()->flywheelIO.setRollerVoltage(voltage.getAsDouble()));
  }

  public Command feed(DoubleSupplier voltage) {
    return this.run(()->flywheelIO.setRollerVoltage(voltage.getAsDouble()));
  }

  private void setHoodVoltage(double hoodVoltage) {
    hoodIO.setHoodVoltage(hoodVoltage);
  }

  private void setHoodPosition(Rotation2d hoodPosition) {
    hoodIO.setHoodPosition(hoodPosition);
  }

  private void setHoodVelocity(double hoodVelocity) {
    hoodIO.setHoodVelocity(hoodVelocity);
  }

  public Command setHoodVoltageCommand(Double hoodVoltage) {
    return this.run(() -> this.setHoodVoltage(hoodVoltage));
  }

  public Command setHoodPositionCommand(Rotation2d hoodPosition) {
    return this.run(() -> this.setHoodPosition(hoodPosition));
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodinputs);
    Logger.processInputs("hood", hoodinputs);

    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("shooter/flywheel", flywheelInputs);
  }
}