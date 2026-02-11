// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.canrange.CANrangeIO;
import frc.robot.components.canrange.CANrangeIOInputsAutoLogged;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/** Lintake = Linear Intake. !! COMP !! */
public class LintakeSubsystem extends SubsystemBase implements Intake {
  // I'm calling zero fully retracted and 1 fully extended (so that kG works if its needed)
  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(16.0);
  public static final double EXTENDED_POSITION_METERS = MAX_EXTENSION_METERS;
  public static final double RACK_GEAR_RATIO = 8.0;
  public static final double RACK_PINION_DIAMETER_METERS = Units.inchesToMeters(0.975);
  public static final double ROLLER_GEAR_RATIO = 34 / 15;
  public static final double CURRENT_ZEROING_THRESHOLD = 30; // TODO: TUNE

  private final LinearRackIO rackIO;
  private LinearRackIOInputsAutoLogged rackIOInputs = new LinearRackIOInputsAutoLogged();

  private final RollerIO rollerIO;
  private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

  private final CANrangeIO canRangeIO;
  private CANrangeIOInputsAutoLogged canRangeIOInputs = new CANrangeIOInputsAutoLogged();

  private LinearFilter rackCurrentFilter = LinearFilter.movingAverage(10);
  private double rackCurrentFilterValue = 0.0;

  /** Creates a new LintakeSubsystem. */
  public LintakeSubsystem(LinearRackIO rackIO, RollerIO rollerIO, CANrangeIO canRangeIO) {
    this.rackIO = rackIO;
    this.rollerIO = rollerIO;
    this.canRangeIO = canRangeIO;
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
          rollerIO.setRollerVoltage(10.0);
        });
  }

  @Override
  public Command outtake() {
    return this.run(
        () -> {
          // Oscillate between 0.5x extension pos and 1x extension pos
          rackIO.setPositionSetpoint(
              (0.25 * Math.sin(Timer.getFPGATimestamp()) + 0.75) * EXTENDED_POSITION_METERS);
          rollerIO.setRollerVoltage(10.0);
        });
  }

  @Override
  public Command rest() {
    return this.run(
        () -> {
          rackIO.setPositionSetpoint(EXTENDED_POSITION_METERS);
          rollerIO.setRollerVoltage(0.0);
        });
  }

  public Command runCurrentZeroing() {
    return this.run(() -> rackIO.setVoltage(3))
        .until(() -> rackCurrentFilterValue > CURRENT_ZEROING_THRESHOLD)
        .andThen(Commands.parallel(Commands.print("Intake Zeroed"), zeroRack()));
  }

  public Command zeroRack() {
    return this.runOnce(() -> rackIO.resetEncoder(MAX_EXTENSION_METERS));
  }

  public static TalonFXConfiguration getRackMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO

    // Converts rotational motion to linear motion
    config.Feedback.SensorToMechanismRatio =
        RACK_GEAR_RATIO * (Math.PI * RACK_PINION_DIAMETER_METERS);

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static; // Maybe don't need this?
    config.Slot0.kG = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kD = 0.0;

    // TODO: TUNE
    config.CurrentLimits.StatorCurrentLimit = 30.0;
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
}
