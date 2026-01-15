package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import frc.robot.components.rollers.RollerIOReal;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  private RollerIOReal io;
  private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public IntakeSubsystem(RollerIOReal io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  // TODO get actual values
  public Command intake() {
    return this.run(
        () -> {
          io.setRollerVoltage(5);
        });
  }

  public Command outake() {
    return this.run(
        () -> {
          io.setRollerVoltage(-2);
        });
  }

  public Command rest() {
    return this.run(
        () -> {
          io.setRollerVoltage(0);
        });
  }
}
