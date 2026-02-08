package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.Command;

public class EmptyClimberSubsystem extends ClimberSubsystem {

  @Override
  public void periodic() {}

  public EmptyClimberSubsystem(CANBus canbus) {
    super(new ClimberIO(canbus));
  }

  @Override
  public Command extendClimber() {
    return this.idle();
  }

  @Override
  public Command retractClimber() {
    return this.idle();
  }
}
