package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Spindexer = Spinning Indexer. !! COMP !! */
public class SpindexerSubsystem extends SubsystemBase implements Indexer {

  public static final double SPINNER_GEAR_RATIO = 67.0 / 15.0;
  public static final double X44_KICKER_GEAR_RATIO = 24.0 / 18.0;
  public static final double X60_KICKER_GEAR_RATIO = 36.0 / 12.0;

  // i don't really know if i should be using the sushi or the stealth wheels but the sushi wheels
  // are 1" in diameter and the stealth wheels are 3" in diameter
  public static final double X44_KICKER_DIAMETER_INCHES = 3;
  // biggest wheel (smallest wheel is 2")
  public static final double SPINNER_DIAMETER_INCHES = 8;

  private RollerIO spinnerIO;

  private RollerIOInputsAutoLogged spinnerInputs = new RollerIOInputsAutoLogged();

  private RollerIO x44KickerIO;
  private RollerIOInputsAutoLogged x44kickerInputs = new RollerIOInputsAutoLogged();

  private RollerIO x60KickerIO;
  private RollerIOInputsAutoLogged x60kickerInputs = new RollerIOInputsAutoLogged();

  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 10.0;

  private final Alert spinnerDisconnectedAlert =
      new Alert("Disconnected spinner motor!", AlertType.kError);
  private final Alert x44KickerDisconnectedAlert =
      new Alert("Disconnected x44 kicker motor!", AlertType.kError);

  private final Alert x60KickerDisconnectedAlert =
      new Alert("Disconnected x60 kicker motor!", AlertType.kError);

  @AutoLogOutput(key = "Kicker/Current Filter Value")
  private double currentFilterValue = 0.0;

  private LinearFilter kickerCurrentFilter = LinearFilter.movingAverage(5);

  public static final double KICKER_CURRENT_THRESHOLD = 20; // TODO

  private LoggedTunableNumber kickerSpeed = new LoggedTunableNumber("Kicker Speed", 100);

  private LoggedTunableNumber spinnerSpeed = new LoggedTunableNumber("Spinner Speed", 13);

  private LoggedTunableNumber x60KickSpeed = new LoggedTunableNumber("X60 Kick Speed", 20);

  private SysIdRoutine x60Sysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Indexer/Kicker X60/SysID State", state.toString())),
          new Mechanism((voltage) -> x60KickerIO.setRollerVoltage(voltage.in(Volts)), null, this));

  private SysIdRoutine x44Sysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Indexer/Kicker X44/SysID State", state.toString())),
          new Mechanism((voltage) -> x44KickerIO.setRollerVoltage(voltage.in(Volts)), null, this));

  public SpindexerSubsystem(
      CANBus canbus, RollerIO indexRollerIO, RollerIO x44KickerIO, RollerIO x60KickerIO) {
    this.x44KickerIO = x44KickerIO;
    this.x60KickerIO = x60KickerIO;

    this.spinnerIO = indexRollerIO;
  }

  @Override
  public Command index() {
    return this.run(
        () -> {
          spinnerIO.setRollerVoltage(7);
          x44KickerIO.setRollerVoltage(-7);
          x60KickerIO.setRollerVoltage(-7);
        });
  }

  @Override
  public Command kick() {
    return Commands.sequence(
        this.run(
            () -> {
              //   spinnerIO.setRollerVoltage(12);
              //   kickerIO.setRollerVoltage(12);
              // })
              //     .withTimeout(3),
              // this.run(
              //     () -> {
              spinnerIO.setRollerVelocity(spinnerSpeed.get());
              x44KickerIO.setRollerVelocity(kickerSpeed.get());
              // x60KickerIO.setRollerVelocity(x60KickSpeed.get());
              x60KickerIO.setRollerVoltage(x60KickSpeed.get());
            }));
  }

  @Override
  public Command spit() {
    return this.run(
        () -> {
          spinnerIO.setRollerVoltage(-7);
          x44KickerIO.setRollerVoltage(-7);
        });
  }

  @Override
  public Command rest() {
    return this.run(
        () -> {
          spinnerIO.setRollerVoltage(0.0);
          x44KickerIO.setRollerVoltage(0.0);
          x60KickerIO.setRollerVoltage(0.0);
        });
  }

  @Override
  public Command stop() {
    return this.run(
        () -> {
          spinnerIO.setRollerVoltage(0.0);
          x44KickerIO.setRollerVoltage(0.0);
        });
  }

  @Override
  public void periodic() {
    spinnerIO.updateInputs(spinnerInputs);
    Logger.processInputs("Indexer/Spinner", spinnerInputs);
    x44KickerIO.updateInputs(x44kickerInputs);
    Logger.processInputs("Indexer/Kicker X44", x44kickerInputs);
    x60KickerIO.updateInputs(x60kickerInputs);
    Logger.processInputs("Indexer/Kicker X60", x60kickerInputs);
    spinnerDisconnectedAlert.set(!spinnerInputs.connected);
    x44KickerDisconnectedAlert.set(!x44kickerInputs.connected);
    x60KickerDisconnectedAlert.set(!x60kickerInputs.connected);

    currentFilterValue = kickerCurrentFilter.calculate(x44kickerInputs.statorCurrentAmps);
  }

  public static TalonFXConfiguration getIndexerConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = SPINNER_GEAR_RATIO;

    config.Slot0.kS = 0.25181;
    config.Slot0.kV = 0.66739;
    config.Slot0.kA = 0.038125;
    config.Slot0.kP = 0.1;
    config.Slot0.kD = 0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    return config;
  }

  public static TalonFXConfiguration getX44KickerConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Converts angular motion to linear motion
    config.Feedback.SensorToMechanismRatio = X44_KICKER_GEAR_RATIO;

    config.Slot0.kS = 0.50822;
    config.Slot0.kV = 0.13022;
    config.Slot0.kA = 0.0051671;
    config.Slot0.kP = 0.5;
    config.Slot0.kD = 0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    return config;
  }

  public static TalonFXConfiguration getX60KickerConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Converts angular motion to linear motion
    config.Feedback.SensorToMechanismRatio = X60_KICKER_GEAR_RATIO;

    config.Slot0.kS = 0.22539;
    config.Slot0.kV = 0.35529;
    config.Slot0.kA = 0.0078104;
    config.Slot0.kP = 1;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    return config;
  }

  public boolean isEmpty() {
    return currentFilterValue < KICKER_CURRENT_THRESHOLD;
  }

  @Override
  public Command runX60Sysid() {
    return Commands.sequence(
        x60Sysid.quasistatic(Direction.kForward),
        x60Sysid.quasistatic(Direction.kReverse),
        x60Sysid.dynamic(Direction.kForward),
        x60Sysid.dynamic(Direction.kReverse));
  }

  @Override
  public Command runX44Sysid() {
    return Commands.sequence(
        x44Sysid.quasistatic(Direction.kForward),
        x44Sysid.quasistatic(Direction.kReverse),
        x44Sysid.dynamic(Direction.kForward),
        x44Sysid.dynamic(Direction.kReverse));
  }
}
