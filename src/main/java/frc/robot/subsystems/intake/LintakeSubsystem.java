// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.components.canrange.CANrangeIO;
import frc.robot.components.canrange.CANrangeIOInputsAutoLogged;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Lintake = Linear Intake. !! COMP !! */
public class LintakeSubsystem extends SubsystemBase implements Intake {
  // I'm calling zero fully retracted and 1 fully extended (so that kG works if its needed)
  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(11.57);
  public static final double EXTENDED_POSITION_METERS = MAX_EXTENSION_METERS;
  // - Units.inchesToMeters(0.1);
  public static final double RACK_GEAR_RATIO = 8.0;
  public static final double RACK_PINION_DIAMETER_METERS = Units.inchesToMeters(0.975);
  public static final double ROLLER_GEAR_RATIO = 34 / 15;
  public static final double CURRENT_ZEROING_THRESHOLD = 35; // TODO: TUNE

  private final LinearRackIO rackIO;
  private LinearRackIOInputsAutoLogged rackIOInputs = new LinearRackIOInputsAutoLogged();

  private final RollerIO rollerIO;
  private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

  private final CANrangeIO canRangeIO;
  private CANrangeIOInputsAutoLogged canRangeIOInputs = new CANrangeIOInputsAutoLogged();

  private LinearFilter rackCurrentFilter = LinearFilter.movingAverage(5);

  @AutoLogOutput(key = "Intake/Current Filter Value")
  private double rackCurrentFilterValue = 0.0;

  private SysIdRoutine intakeRollerSysid;

  private SysIdRoutine extensionSysid;

  /** Creates a new LintakeSubsystem. */
  public LintakeSubsystem(LinearRackIO rackIO, RollerIO rollerIO, CANrangeIO canRangeIO) {
    this.rackIO = rackIO;
    this.rollerIO = rollerIO;
    this.canRangeIO = canRangeIO;

    intakeRollerSysid =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/Rollers/SysID State", state.toString())),
            new Mechanism((volts) -> rollerIO.setRollerVoltage(volts.in(Volts)), null, this));

    extensionSysid =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/Extension/SysID State", state.toString())),
            new Mechanism((voltage) -> rackIO.setVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    rackIO.updateInputs(rackIOInputs);
    Logger.processInputs("Intake/Rack", rackIOInputs);

    rollerIO.updateInputs(rollerIOInputs);
    Logger.processInputs("Intake/Rollers", rollerIOInputs);

    canRangeIO.updateInputs(canRangeIOInputs);
    Logger.processInputs("Intake/CANRange", canRangeIOInputs);

    rackCurrentFilterValue = rackCurrentFilter.calculate(rackIOInputs.statorCurrentAmps);
  }

  @Override
  public Command intake() {
    return this.run(
        () -> {
          rackIO.setPositionSetpoint(EXTENDED_POSITION_METERS);
          rollerIO.setRollerVoltage(7.0);
        });
  }

  @Override
  public Command outtake() {
    return this.run(
        () -> {
          // Should oscillate between 0.8x extension pos and 1x extension pos
          rackIO.setPositionSetpoint(
              (0.1 * Math.sin(Timer.getFPGATimestamp() * 6) + 0.9) * EXTENDED_POSITION_METERS);
          rollerIO.setRollerVoltage(10.0);
        });
  }

  @Override
  public Command restExtended() {
    return this.run(
        () -> {
          rackIO.setPositionSetpoint(EXTENDED_POSITION_METERS);
          rollerIO.setRollerVoltage(0.0);
        });
  }

  public Command runCurrentZeroing() {
    // return this.run(() -> rackIO.setVoltage(-5))
    //     .until(
    //         new Trigger(() -> Math.abs(rackCurrentFilterValue) > CURRENT_ZEROING_THRESHOLD)
    //             .debounce(0.95))
    //     .andThen(Commands.parallel(Commands.print("Intake Zeroed"), zeroRack()));
    return Commands.deadline(
            Commands.waitSeconds(0.5)
                .andThen(
                    Commands.waitUntil(
                        () -> Math.abs(rackCurrentFilterValue) > CURRENT_ZEROING_THRESHOLD)),
            this.run(() -> rackIO.setVoltage(-3)))
        .andThen(Commands.parallel(Commands.print("Intake Zeroed"), zeroRack()));
  }

  public Command zeroRack() {
    return this.runOnce(() -> rackIO.resetEncoder(0));
  }

  public static TalonFXConfiguration getRackMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Converts rotational motion to linear motion
    config.Feedback.SensorToMechanismRatio =
        RACK_GEAR_RATIO / (Math.PI * RACK_PINION_DIAMETER_METERS);

    // config.Slot0.GravityType = GravityTypeValue.Elevator_Static; // Maybe don't need this?
    // config.Slot0.kG = 0.0;
    config.Slot0.kS = 0.771;
    config.Slot0.kV = 7.51;
    config.Slot0.kA = 1.2;

    config.Slot0.kP = 420.0;

    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // TODO: TUNE
    config.MotionMagic.MotionMagicCruiseVelocity = 10.0;
    config.MotionMagic.MotionMagicAcceleration = 30.0;

    return config;
  }

  public static TalonFXConfiguration getRollerMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO

    // Converts rotational motion to linear motion
    config.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kD = 0.0;

    // TODO: TUNE
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }

  @Override
  public boolean beambreak() {
    return canRangeIOInputs.isDetected;
  }

  @Override
  public Command runRollerSysid() {
    return Commands.sequence(
        intakeRollerSysid.quasistatic(Direction.kForward),
        intakeRollerSysid.quasistatic(Direction.kReverse),
        intakeRollerSysid.dynamic(Direction.kForward),
        intakeRollerSysid.dynamic(Direction.kReverse));
  }

  @Override
  public Command runExtensionSysid() {
    return Commands.sequence(
        extensionSysid
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    rackIOInputs.positionMeters
                        > (MAX_EXTENSION_METERS - Units.inchesToMeters(1))), // Stop before endstop
        extensionSysid
            .quasistatic(Direction.kReverse)
            .until(() -> rackIOInputs.positionMeters < Units.inchesToMeters(1)),
        extensionSysid
            .dynamic(Direction.kForward)
            .until(
                () ->
                    rackIOInputs.positionMeters > (MAX_EXTENSION_METERS - Units.inchesToMeters(1))),
        extensionSysid
            .dynamic(Direction.kReverse)
            .until(() -> rackIOInputs.positionMeters < Units.inchesToMeters(1)));
  }

  @Override
  public Command climb() {
    return this.run(() -> rackIO.setPositionSetpoint(0));
  }

  @Override
  public Command restRetracted() {
    return this.run(
        () -> {
          rackIO.setPositionSetpoint(0);
          rollerIO.setRollerVoltage(0.0);
        });
  }
}
