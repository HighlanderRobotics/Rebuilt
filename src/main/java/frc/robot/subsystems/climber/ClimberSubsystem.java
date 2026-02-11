package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  // From CAD
  public static final double SPOOL_DIAMETER_METERS = Units.inchesToMeters(0.668898);
  // todo: find actual constants
  public static double GEAR_RATIO = (45.0 / 1.0);
  public static double MAX_EXTENSION_METERS = 0.2413;
  public static double MAX_ACCELERATION = 10.0;
  public static double MAX_VELOCITY = 2.0;

  ClimberIO climberIO;
  ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  
  private SysIdRoutine climberSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Climber/SysID State", state.toString())),
          new Mechanism((voltage) -> climberIO.setClimberVoltage(voltage.in(Volts)), null, this));

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

    public Command runClimberSysid() { //TODO after climber unit fix
    return Commands.sequence(
        climberSysid
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    climberInputs.motorPositionMeters
                        > (MAX_EXTENSION_METERS - Units.inchesToMeters(1))), // Stop before endstop
        climberSysid
            .quasistatic(Direction.kReverse)
            .until(
                () ->
                    climberInputs.motorPositionMeters
                        < Units.inchesToMeters(1)),
        climberSysid
            .dynamic(Direction.kForward)
            .until(
                () ->
                    climberInputs.motorPositionMeters
                        > (MAX_EXTENSION_METERS - Units.inchesToMeters(1))),
        climberSysid
            .dynamic(Direction.kReverse)
            .until(
                () ->
                    climberInputs.motorPositionMeters
                        < Units.inchesToMeters(1)));
  }
}
