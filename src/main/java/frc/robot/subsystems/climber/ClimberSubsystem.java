package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  // todo: find actual constants
  public static double GEAR_RATIO = (45.0 / 1.0);
  public static double MAX_EXTENSION_METERS = 0.2413;
  public static double SPOOL_DIAMETER_METERS = Units.inchesToMeters(0.668898);
  public static double MAX_ACCELERATION = 10.0;
  public static double MAX_VELOCITY = 2.0;

  ClimberIO climberIO;
  ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber", climberInputs);
  }

  // member variables here?

  public ClimberSubsystem(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  public Command extendClimber() {
    return this.run(
        () -> {
          climberIO.setClimberPosition(MAX_EXTENSION_METERS);
        });
  }

  public Command retractClimber() {
    return this.run(
        () -> {
          climberIO.setClimberPosition(0.0);
        });
  }

  public double getClimberExtensionMeters() {
    // Convert rotations into linear motion
    return climberInputs.motorPositionRotations.getRotations() * (Math.PI * SPOOL_DIAMETER_METERS);
  }
}
