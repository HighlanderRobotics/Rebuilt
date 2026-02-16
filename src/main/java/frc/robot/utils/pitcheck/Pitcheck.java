// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.pitcheck;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.LintakeSubsystem;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class Pitcheck {

  LintakeSubsystem intake = new LintakeSubsystem(null, null, null);
  //   SpindexerSubsystem spindexer = new SpindexerSubsystem();
  BooleanSupplier intakeRunningForward =
      () -> intake.getRollerVoltage() > 9.0 && intake.getRollerVoltage() < 11.0;
  BooleanSupplier intakeRunningBackward =
      () -> intake.getRollerVoltage() < -9.0 && intake.getRollerVoltage() > -11.0;
  BooleanSupplier intakeRest = () -> (intake.getRollerVoltage()) == 0.0;

  public void pitcheck() {
    SmartDashboard.putData(
        "intakeRoller",
        Commands.sequence(
            pitCheck(intake.intake(), intakeRunningForward),
            pitCheck(intake.outtake(), intakeRunningBackward),
            pitCheck(intake.rest(), intakeRest)));
    // SmartDashboard.putData(
    //     "spindexer",
    //     new SequentialCommandGroup(pitCheck(spindexer, spindexer.kick(), spindexerKick)));
  }

  private Command pitCheck(Command command, BooleanSupplier endstate) {
    final boolean[] success = {false};

    return Commands.sequence(
        Commands.runOnce(() -> success[0] = false),
        command
            .until(
                () -> {
                  if (endstate.getAsBoolean()) {
                    success[0] = true;
                    return true;
                  }
                  return false;
                })
            .withTimeout(2.0),
        Commands.runOnce(() -> System.out.println("Pitcheck success: " + endstate.getAsBoolean())));
  }
}
