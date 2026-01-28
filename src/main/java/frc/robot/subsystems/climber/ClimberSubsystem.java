package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;



public class ClimberSubsystem extends SubsystemBase {
  //todo: find actual constants
  public static final double GEAR_RATIO = (45.0 / 1.0);
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 2.0;

ClimberIO climberIO;
ClimberIOInputsAutoLogged ClimberIOInputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
  }

//member variables here?

public ClimberSubsystem(ClimberIO climberIO) {
  this.climberIO = climberIO;
}

//not sure about these implementations, some issues with "static reference to non-static method"
public Command climbUp() {
  return this.run(
    () -> {
      ClimberIO.setClimberPosition(MAX_ANGLE)
    });
    
}

public Command climbDown() {
  return this.run(
    () -> {
      ClimberIO.setClimberPosition(MIN_ANGLE)
    });
    
}

}
