package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  public static final double GEAR_RATIO = 2.0;

  private RollerIO io;
  private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private SysIdRoutine intakeRollerSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Intake/SysID State", state.toString())),
          new Mechanism((volts) -> io.setRollerVoltage(volts.in(Volts)), null, this));

  public IntakeSubsystem(RollerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  // TODO get actual values
  public Command intake() {
    return this.run(() -> io.setRollerVoltage(10));
  }

  public Command outake() {
    return this.run(() -> io.setRollerVoltage(-5));
  }

  public Command rest() {
    return this.run(() -> io.setRollerVoltage(0));
  }

  public Command runRollerSysid() {
    return Commands.sequence(
        intakeRollerSysid.quasistatic(Direction.kForward),
        intakeRollerSysid.quasistatic(Direction.kReverse),
        intakeRollerSysid.dynamic(Direction.kForward),
        intakeRollerSysid.dynamic(Direction.kReverse));
  }

  public static TalonFXConfiguration getIntakeConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.Slot0.kS = 0.42;
    config.Slot0.kV = 0.21;
    config.Slot0.kA = 0.00347;

    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }
}
