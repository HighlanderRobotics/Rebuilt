package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  // From CAD
  public static final double SPOOL_DIAMETER_METERS = Units.inchesToMeters(1.0);
  // todo: find actual constants
  public static double GEAR_RATIO = (45.0 / 1.0);
  public static double MAX_EXTENSION_METERS = 0.16748 + Units.inchesToMeters(2);
  public static double MAX_ACCELERATION = 10.0;
  public static double MAX_VELOCITY = 2.0;

  ClimberIO climberIO;
  ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  // turned off climber

  private SysIdRoutine climberSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Climber/SysID State", state.toString())),
          new Mechanism((voltage) -> climberIO.setClimberVoltage(voltage.in(Volts)), null, this));

  private double currentFilterValue = 0.0;
  private LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private static final double CURRENT_ZERO_THRESHOLD = 30;

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber", climberInputs);
    currentFilterValue = currentFilter.calculate(climberInputs.motorStatorCurrentAmps);
  }

  // member variables here?

  public ClimberSubsystem(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  public Command extend() {
    return this.run(
        () -> {
          Commands.none();
          //  climberIO.setClimberPosition(MAX_EXTENSION_METERS);
        });
  }

  public Command retract() {
    return this.run(
        () -> {
          Commands.none();
          // climberIO.setClimberPosition(Units.inchesToMeters(1));
        });
  }

  public Command zeroClimber() {
    return this.runOnce(() -> Commands.none());
    // climberIO.resetEncoder(0.0)).ignoringDisable(true);
  }

  public Command runClimberSysid() {
    return Commands.sequence(
        climberSysid
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    climberInputs.motorPositionMeters
                        > (MAX_EXTENSION_METERS - Units.inchesToMeters(1))), // Stop before endstop
        climberSysid
            .quasistatic(Direction.kReverse)
            .until(() -> climberInputs.motorPositionMeters < Units.inchesToMeters(1)),
        climberSysid
            .dynamic(Direction.kForward)
            .until(
                () ->
                    climberInputs.motorPositionMeters
                        > (MAX_EXTENSION_METERS - Units.inchesToMeters(1))),
        climberSysid
            .dynamic(Direction.kReverse)
            .until(() -> climberInputs.motorPositionMeters < Units.inchesToMeters(1)));
  }

  public Command runCurrentZeroing() {
    return Commands.none();
    //  climberIO.setClimberVoltage(-3.0))
    //     .until(new Trigger(() -> Math.abs(currentFilterValue) > CURRENT_ZERO_THRESHOLD))
    //     .andThen(Commands.parallel(Commands.print("Climber Zeroed"), zeroClimber()));
  }
}
