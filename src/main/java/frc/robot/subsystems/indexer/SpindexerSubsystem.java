package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.utils.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Spindexer = Spinning Indexer. !! COMP !! */
public class SpindexerSubsystem extends SubsystemBase implements Indexer {

  public static final double GEAR_RATIO = 2.0;
  // i don't really know if i should be using the sushi or the stealth wheels but the sushi wheels
  // are 1" in diameter and the stealth wheels are 3" in diameter
  public static final double KICKER_DIAMETER_INCHES = 3;
  // biggest wheel (smallest wheel is 2")
  public static final double SPINNER_DIAMETER_INCHES = 8;

  private RollerIO spinnerIO;

  RollerIOInputsAutoLogged spinnerInputs = new RollerIOInputsAutoLogged();

  RollerIO kickerIO;
  RollerIOInputsAutoLogged kickerInputs = new RollerIOInputsAutoLogged();

  private SysIdRoutine indexRollerSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Indexer/Roller/SysID State", state.toString())),
          new Mechanism((volts) -> spinnerIO.setRollerVoltage(volts.in(Volts)), null, this));

  private SysIdRoutine kickerSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Indexer/Kicker/SysID State", state.toString())),
          new Mechanism((volts) -> kickerIO.setRollerVoltage(volts.in(Volts)), null, this));

  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 10.0;
  public static final double KICKER_GEAR_RATIO = 2.0;

  private LoggedTunableNumber testKickVolts = new LoggedTunableNumber("Indexer/Kicker Voltage", 10);
  private LoggedTunableNumber testSpinVolts = new LoggedTunableNumber("Indexer/Spinner Voltage", 8);

  private final Alert spinnerDisconnectedAlert =
      new Alert("Disconnected spinner motor!", AlertType.kError);
  private final Alert kickerDisconnectedAlert =
      new Alert("Disconnected kicker motor!", AlertType.kError);

  public SpindexerSubsystem(CANBus canbus, RollerIO indexRollerIO, RollerIO kickerIO) {
    this.kickerIO = kickerIO;
    this.spinnerIO = indexRollerIO;
  }

  @Override
  public Command index() {
    return this.run(
        () -> {
          spinnerIO.setRollerVoltage(7);
          kickerIO.setRollerVoltage(-7);
        });
  }

  @Override
  public Command kick(DoubleSupplier flywheelSpeedSupplier) {
    return Commands.sequence(
        this.run(
            () -> {
              double surfaceSpeedInPerSec =
                  flywheelSpeedSupplier.getAsDouble()
                      * Math.PI
                      * TurretSubsystem.FLYWHEEL_DIAMETER_INCHES;
              double kickerSpeed = surfaceSpeedInPerSec / (Math.PI * KICKER_DIAMETER_INCHES);
              // arbitrarily deciding to have it match the bottom wheel although i have no clue
              // if
              // that's right
              double spinnerSpeed = surfaceSpeedInPerSec / (Math.PI * SPINNER_DIAMETER_INCHES);
              Logger.recordOutput("Indexer/Spinner/Adjusted speed", spinnerSpeed);
              Logger.recordOutput("Indexer/Kicker/Adjusted speed", kickerSpeed);
              spinnerIO.setRollerVelocity(spinnerSpeed);
              kickerIO.setRollerVelocity(kickerSpeed);
              // spinnerIO.setRollerVelocity(20);
              // kickerIO.setRollerVelocity(15);
            })
        //     .withTimeout(3),
        // this.run(
        //     () -> {
        //       spinnerIO.setRollerVelocity(30);
        //       kickerIO.setRollerVelocity(20);
        //     })
        );
  }

  @Override
  public Command spit() {
    return this.run(
        () -> {
          spinnerIO.setRollerVoltage(-7);
          kickerIO.setRollerVoltage(-7);
        });
  }

  @Override
  public Command rest() {
    return this.run(
        () -> {
          spinnerIO.setRollerVoltage(0.0);
          kickerIO.setRollerVoltage(0.0);
        });
  }

  public static TalonFXConfiguration getIndexerConfigs() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.Slot0.kS = 0.12567;
    config.Slot0.kV = 0.23782;
    config.Slot0.kA = 0.019071;
    config.Slot0.kP = 0.1;
    config.Slot0.kD = 0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    return config;
  }

  public static TalonFXConfiguration getKickerConfigs() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Converts angular motion to linear motion
    config.Feedback.SensorToMechanismRatio = KICKER_GEAR_RATIO;

    config.Slot0.kS = 0.41787;
    config.Slot0.kV = 0.26065;
    config.Slot0.kA = 0.029144;
    config.Slot0.kP = 7;
    config.Slot0.kD = 0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    return config;
  }

  @Override
  public void periodic() {
    spinnerIO.updateInputs(spinnerInputs);
    Logger.processInputs("Indexer/Spinner", spinnerInputs);
    kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Indexer/Kicker", kickerInputs);
    spinnerDisconnectedAlert.set(!spinnerInputs.connected);
    kickerDisconnectedAlert.set(!kickerInputs.connected);
  }

  public Command runRollerSysId() {
    return Commands.sequence(
        indexRollerSysid.quasistatic(Direction.kForward),
        indexRollerSysid.quasistatic(Direction.kReverse),
        indexRollerSysid.dynamic(Direction.kForward),
        indexRollerSysid.dynamic(Direction.kReverse));
  }

  @Override
  public Command runKickerSysId() {
    return Commands.sequence(
        kickerSysid.quasistatic(Direction.kForward),
        kickerSysid.quasistatic(Direction.kReverse),
        kickerSysid.dynamic(Direction.kForward),
        kickerSysid.dynamic(Direction.kReverse));
  }

  @Override
  public Command testShoot() {
    return this.run(
        () -> {
          kickerIO.setRollerVoltage(testKickVolts.get());
          spinnerIO.setRollerVoltage(testSpinVolts.get());
        });
  }
}
