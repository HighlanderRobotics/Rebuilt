package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.CANBus;

public class EmptyClimberSubsystem extends ClimberSubsystem{

  @Override
  public void periodic(){}

public EmptyClimberSubsystem(CANBus canbus) {
    super(new ClimberIO(canbus));
}

public Command climbUp() {
    return this.idle();
    }

public Command climbDown() {
    return this.idle();
    }
}
