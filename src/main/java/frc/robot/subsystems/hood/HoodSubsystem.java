// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import com.google.common.base.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
  private HoodIO hoodIO;
  private HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private SysIdRoutine hoodSysid = new SysIdRoutine(new Config(), new Mechanism((voltage) -> hoodIO.setHoodVoltage(voltage.in(Volts)), null, this));

  public static double GEAR_RATIO = 147.0 / 13.0;

  public static Rotation2d MAX_ROTATION = Rotation2d.fromDegrees(90); // TODO: ACTUAL VALUE
  public static Rotation2d MIN_ROTATION = Rotation2d.fromDegrees(90); // TODO: ACTUAL VALUE


  /** Creates a new HoodSubsystem. */
  public HoodSubsystem(HoodIO io) {
    this.hoodIO = io;
  }

  public Command setHoodPositionCommand(Supplier<Rotation2d> hoodPosition) {
    return this.run(() -> hoodIO.setHoodPosition(hoodPosition.get()));
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
  }

  public Command runHoodSysid() {
    return Commands.sequence(
      hoodSysid.quasistatic(Direction.kForward).until(() -> hoodInputs.hoodPositionRotations.getDegrees() > (MAX_ROTATION.getDegrees() - 5)), // Stop before endstop
      hoodSysid.quasistatic(Direction.kReverse).until(() -> hoodInputs.hoodPositionRotations.getDegrees() > (MIN_ROTATION.getDegrees() + 5)),
      hoodSysid.dynamic(Direction.kForward).until(() -> hoodInputs.hoodPositionRotations.getDegrees() > (MAX_ROTATION.getDegrees() - 5)),
      hoodSysid.dynamic(Direction.kReverse).until(() -> hoodInputs.hoodPositionRotations.getDegrees() > (MIN_ROTATION.getDegrees() + 5))
    );
  }
}
