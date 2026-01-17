// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOInputsAutoLogged;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  HoodIO hoodIO;
  HoodIOInputsAutoLogged hoodinputs = new HoodIOInputsAutoLogged();

  public static double GEAR_RATIO = 147.0 / 13.0;

  FlywheelIO flywheelIO;
  FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  /** Creates a new HoodSubsystem. */
  public ShooterSubsystem(HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;
  }

  public Command shoot(DoubleSupplier voltage) {
    return this.run(() -> flywheelIO.setFlywheelVoltage(voltage.getAsDouble()));
  }

  public Command feed(DoubleSupplier voltage) {
    return this.run(() -> flywheelIO.setFlywheelVoltage(voltage.getAsDouble()));
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
    Logger.processInputs("shooter/hood", hoodinputs);

    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("shooter/flywheel", flywheelInputs);
  }
}
