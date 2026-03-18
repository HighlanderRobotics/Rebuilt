package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  // From CAD
  public static final double SPOOL_DIAMETER_METERS = Units.inchesToMeters(1.0);
  // todo: find actual constants
  public static double GEAR_RATIO = (45.0 / 1.0);
  public static double MAX_EXTENSION_METERS =
      0.16748 + Units.inchesToMeters(2 - 0.75 + 0.625 + 0.25);
  public static double MAX_ACCELERATION = 10.0;
  public static double MAX_VELOCITY = 2.0;

  ClimberIO io;
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  @AutoLogOutput(key = "Climber/Current Filter Value")
  private double currentFilterValue = 0.0;

  private LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private static final double CURRENT_ZERO_THRESHOLD =
      7; // Might want to raise this but worked at lab
  private final Alert climberDisconnectedAlert =
      new Alert("Disconnected climber motor!", AlertType.kError);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);
    climberDisconnectedAlert.set(!inputs.connected);
  }

  // member variables here?

  public ClimberSubsystem(ClimberIO climberIO) {
    this.io = climberIO;
    io.resetEncoder(Units.inchesToMeters(1));
  }

  public Command extend() {
    return this.run(
        () -> {
          io.setClimberPosition(MAX_EXTENSION_METERS);
        });
  }

  public Command retract() {
    return this.run(
        () -> {
          io.setClimberPosition(Units.inchesToMeters(1));
        });
  }

  public Command zeroClimber() {
    return this.runOnce(() -> io.resetEncoder(0.0)).ignoringDisable(true);
  }

  public Command runCurrentZeroing() {
    return this.run(() -> io.setClimberVoltage(-0.5))
        .until(new Trigger(() -> Math.abs(currentFilterValue) > CURRENT_ZERO_THRESHOLD))
        .andThen(Commands.parallel(Commands.print("Climber Zeroed"), zeroClimber()));
  }

  public double getClimberExtensionMeters() {
    // Convert rotations into linear motion
    return inputs.positionMeters;
  }

  public double getClimberSetpointMeters() {
    return io.getClimberSetpointMeters();
  }

  public boolean atFullExtension() {
    return MathUtil.isNear(
        getClimberExtensionMeters(), MAX_EXTENSION_METERS, Units.inchesToMeters(1));
  }
}
