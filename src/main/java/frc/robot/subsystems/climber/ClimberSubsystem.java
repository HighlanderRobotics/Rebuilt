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
  public static double GEAR_RATIO = (45.0 / 1.0);
  public static double MAX_EXTENSION_METERS = 0.2413;
  public static double MAX_ACCELERATION = 10.0;
  public static double MAX_VELOCITY = 2.0;

ClimberIO climberIO;
ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
  Logger.processInputs("Climber", climberInputs);
  }

//member variables here?

public ClimberSubsystem(ClimberIO climberIO) {
  this.climberIO = climberIO;
}

public Command climbUp() {
  return this.run(
    () -> {
      climberIO.setClimberPosition(MAX_EXTENSION_METERS);
      wait(1000);
      climberIO.setClimberPosition(0.0);
      // TODO figure out how to correctly implement
    });
    
}

public Command climbDown() {
 return this.run(
    () -> {
      climberIO.setClimberPosition(MAX_EXTENSION_METERS);
      wait(1000);
      climberIO.setClimberPosition(0.0);
      // TODO figure out how to correctly implement
    });
    
}

}
