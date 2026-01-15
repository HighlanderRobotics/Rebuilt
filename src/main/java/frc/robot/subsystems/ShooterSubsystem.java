package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  public Command shoot() {
    return this.idle();
  }

  public Command feed() {
    return this.idle();
  }

  public Command rest() {
    return this.idle();
  }
}
