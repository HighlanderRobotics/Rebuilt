// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.pitcheck;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.LintakeSubsystem;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class Pitcheck {

  LintakeSubsystem intake = new LintakeSubsystem(null, null, null);
  SpindexerSubsystem spindexer = new SpindexerSubsystem(null, null, null);
  BooleanSupplier intakeRunningForward = () -> MathUtil.isNear(7.0, intake.getRollerVoltage(), 1.0);
  BooleanSupplier intakeRunningBackward =
      () -> MathUtil.isNear(-11.0, intake.getRollerVoltage(), 1.0);
  BooleanSupplier intakeRest = () -> MathUtil.isNear(0.0, intake.getRollerVoltage(), 0.5);
  BooleanSupplier spindexerRunningForward =
      () -> MathUtil.isNear(7.0, spindexer.getRollerVoltage(), 1.0);

  public void pitcheck() {
    SmartDashboard.putData(
        "intakeRoller",
        Commands.sequence(
            pitCheck(intake.intake(), intakeRunningForward),
            pitCheck(intake.outtake(), intakeRunningBackward),
            pitCheck(intake.rest(), intakeRest)));
    SmartDashboard.putData(
        "spindexer", Commands.sequence(pitCheck(spindexer.kick(), spindexerRunningForward)));
  }

  private Command pitCheck(Command command, BooleanSupplier endstate) {

    return command
        .until(
            () -> {
              if (endstate.getAsBoolean()) {

                return true;
              }
              return false;
            })
        .withTimeout(2.0)
        .andThen(() -> System.out.println("Pitcheck success: " + endstate.getAsBoolean()));
  }
}
