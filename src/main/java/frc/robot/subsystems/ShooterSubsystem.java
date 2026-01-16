package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {
  public Command shoot() {
    return this.idle();
  }

  public Command feed() {
    return this.idle();
  }

  public Command rest() {
    return this.idle();
  }

  @Override
  public void close() throws Exception {
      // Currently nothing to close
  }
}
