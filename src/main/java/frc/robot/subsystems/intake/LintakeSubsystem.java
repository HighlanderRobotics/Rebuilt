// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/** Lintake = Linear Intake. !! COMP !! */
public class LintakeSubsystem extends SubsystemBase implements Intake {
  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(16.0);
  public static final double EXTENDED_POSITION_METERS = MAX_EXTENSION_METERS;
  public static final double RACK_GEAR_RATIO = 8.0;
  public static final double RACK_PINION_DIAMETER_METERS = Units.inchesToMeters(0.975);

  private final LinearSlideIO rackIO;
  private LinearSlideIOInputsAutoLogged rackIOInputs = new LinearSlideIOInputsAutoLogged();

  private final RollerIO rollerIO;
  private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

  /** Creates a new LintakeSubsystem. */
  public LintakeSubsystem(LinearSlideIO rackIO, RollerIO rollerIO) {
    this.rackIO = rackIO;
    this.rollerIO = rollerIO;
  }

  @Override
  public void periodic() {
    rackIO.updateInputs(rackIOInputs);
    Logger.processInputs("Intake/Rack", rackIOInputs);

    rollerIO.updateInputs(rollerIOInputs);
    Logger.processInputs("Intake/Rollers", rollerIOInputs);
  }

  @Override
  public Command intake() {
    return this.run(
        () -> {
          rackIO.setPositionSetpoint(EXTENDED_POSITION_METERS);
          rollerIO.setRollerVoltage(10.0);
        });
  }

  @Override
  public Command outtake() {
    return this.run(
        () -> {
          rackIO.setPositionSetpoint(EXTENDED_POSITION_METERS);
          rollerIO.setRollerVoltage(10.0);
        });
  }

  @Override
  public Command rest() {
    return this.run(
        () -> {
          rackIO.setPositionSetpoint(EXTENDED_POSITION_METERS);
          rollerIO.setRollerVoltage(0.0);
        });
  }
}
