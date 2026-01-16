package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO
public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public Command intake() {
    // TODO
    return idle();
  }

  // Can't call it idle bc idle is smthing else
  public Command rest() {
    // TODO
    return idle();
  }

  public Command spit() {
    // TODO
    return idle();
  }

  @Override
  public void close() throws Exception {
    // Currently does nothing bc nothing to close
  }
}
