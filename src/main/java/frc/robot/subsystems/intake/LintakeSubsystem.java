// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/** Lintake = Linear Intake. !! COMP !! */
public class LintakeSubsystem extends SubsystemBase implements Intake {

  private ExtensionIO io;
  // private RollerIO io;
  private ExtensionIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  // private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private SysIdRoutine ExtensionSysid =
      new SysIdRoutine(
          new Config(
              null, null, null, (state) -> Logger.recordOutput("Extension/SysID State", state)),
          new Mechanism((volts) -> io.setExtensionVoltage(volts.in(Volts)), null, this));

  public void ExtensionSubsystem(ExtensionIO io) {
    this.io = io;
  }

  // TODO get actual values
  public Command extend() {
    return this.run(() -> io.setExtensionVoltage(5));
  }

  public Command contract() {
    return this.run(() -> io.setExtensionVoltage(-2));
  }

  public Command stop() {
    return this.run(() -> io.setExtensionVoltage(0));
  }

  public Command runExtensionSysid() {
    return Commands.sequence(
        ExtensionSysid.quasistatic(Direction.kForward),
        ExtensionSysid.quasistatic(Direction.kReverse),
        ExtensionSysid.dynamic(Direction.kForward),
        ExtensionSysid.dynamic(Direction.kReverse));
  }

  /** Creates a new LintakeSubsystem. */
  public LintakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Extension", inputs);
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
}
