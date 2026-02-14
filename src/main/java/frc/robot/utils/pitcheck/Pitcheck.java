// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.pitcheck;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.indexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.LintakeSubsystem;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class Pitcheck {

  LintakeSubsystem intake = new LintakeSubsystem();
  SpindexerSubsystem spindexer = new SpindexerSubsystem();
  BooleanSupplier intakeRunningForward =
      () -> intake.getRollerVoltage() > 9.0 && intake.getRollerVoltage() < 11.0;
  BooleanSupplier intakeRunningBackward =
      () -> intake.getRollerVoltage() < -9.0 && intake.getRollerVoltage() > -11.0;
  BooleanSupplier intakeRest = () -> intake.getRollerVoltage() == 0;

  public void pitcheck(Subsystem subsystem, Command command, BooleanSupplier endState) {
    SmartDashboard.putData(
        "intakeRoller",
        new SequentialCommandGroup(
            pitCheck(intake, intake.intake(), intakeRunningForward),
            pitCheck(intake, intake.outtake(), intakeRunningBackward),
            pitCheck(intake, intake.rest(), intakeRest)));
    SmartDashboard.putData(
        "spindexer",
        new SequentialCommandGroup(pitCheck(spindexer, spindexer.kick(), spindexerKick)));
  }

  private Command pitCheck(Subsystem subsystem, Command command, BooleanSupplier endstate) {

    return command.until(endstate);
  }
}
