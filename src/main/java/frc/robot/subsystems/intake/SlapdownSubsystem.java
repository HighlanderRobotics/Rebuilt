package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.components.cancoder.CANcoderIO;
import frc.robot.components.cancoder.CANcoderIOInputsAutoLogged;
import frc.robot.components.cancoder.CANcoderIOSim;
import frc.robot.components.pivot.PivotIO;
import frc.robot.components.pivot.PivotIOInputsAutoLogged;
import frc.robot.components.rollers.RollerIO;
import frc.robot.components.rollers.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SlapdownSubsystem extends SubsystemBase implements Intake {
  public static final Rotation2d PIVOT_MIN_POSITION =
      Rotation2d.fromDegrees(-26.894531); // Rotation2d.fromRotations(-0.052002);
  public static final Rotation2d PIVOT_MAX_POSITION =
      Rotation2d.fromDegrees(102.568359);//115); // Not so sure abt this one...
  public static final Rotation2d PIVOT_EXTENDED_POSITION = PIVOT_MIN_POSITION;
  public static final Rotation2d PIVOT_RETRACTED_POSITION = PIVOT_MAX_POSITION;
  public static final double CURRENT_ZEROING_THRESHOLD = 30.0; // TODO: TUNE
  public static final double ROLLER_GEAR_RATIO = 60.0 / 29.0;
  public static final double PIVOT_GEAR_RATIO = 36.17578125; // 39.375;

  private final PivotIO pivotIO;
  private PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();

  private final CANcoderIO cancoderIO;
  private CANcoderIOInputsAutoLogged cancoderIOInputs = new CANcoderIOInputsAutoLogged();

  private final RollerIO rollerIO;
  private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

  private Trigger atExtensionTrigger = new Trigger(this::atExtension).debounce(0.05);

  private LinearFilter currentFilter = LinearFilter.movingAverage(5);

  @AutoLogOutput(key = "Intake/Pivot/Current Filter Value")
  private double currentFilterValue = 0.0;

  public SlapdownSubsystem(PivotIO pivotIO, CANcoderIO cancoderIO, RollerIO rollerIO) {
    this.pivotIO = pivotIO;
    this.cancoderIO = cancoderIO;
    this.rollerIO = rollerIO;
  }

  @Override
  public void slapdownInit() {
    pivotIO.resetEncoder(cancoderIOInputs.cancoderPositionRotations);
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotIOInputs);
    Logger.processInputs("Intake/Pivot", pivotIOInputs);

    cancoderIO.updateInputs(cancoderIOInputs);
    Logger.processInputs("Intake/CANcoder", cancoderIOInputs);

    rollerIO.updateInputs(rollerIOInputs);
    Logger.processInputs("Intake/Roller", rollerIOInputs);

    // Log setpoint
    Logger.recordOutput("Intake/Pivot/Setpoint", pivotIO.getSetpoint());

    currentFilterValue = currentFilter.calculate(pivotIOInputs.statorCurrentAmps);
  }

  @Override
  public void simulationPeriodic() {
    // Safe type cast
    if (cancoderIO instanceof CANcoderIOSim) {
      // This does get called after periodic so should have up-to-date info
      ((CANcoderIOSim) cancoderIO)
          .setSimValues(pivotIOInputs.positionRotations); // I assume this is how u do this
    }
  }

  @Override
  public Command agitate() {
    return Commands.sequence(
            this.run(
                    () -> {
                      // maybe needs to go slower but idrk how to do that rn
                      pivotIO.setMotorPositionSetpoint(PIVOT_EXTENDED_POSITION);
                      rollerIO.setRollerVelocity(30.0);
                    })
                .until(atExtensionTrigger),
            this.run(
                    () -> {
                      pivotIO.setMotorPositionSetpoint(
                          PIVOT_EXTENDED_POSITION.plus(Rotation2d.fromDegrees(40)));
                      rollerIO.setRollerVelocity(30.0);
                    })
                .until(atExtensionTrigger))
        .repeatedly();
    // );
  }

  @Override
  public Command intake() {
    return this.run(
            () -> {
              pivotIO.setMotorPositionSetpoint(PIVOT_EXTENDED_POSITION);
              rollerIO.setRollerVelocity(80);
            })
        .until(atExtensionTrigger)
        .andThen(
            this.run(
                () -> {
                  pivotIO.setMotorVoltage(0);
                  rollerIO.setRollerVelocity(80);
                }));
  }

  @Override
  public Command outtake() {
    return this.run(
        () -> {
          pivotIO.setMotorPositionSetpoint(PIVOT_EXTENDED_POSITION);
          rollerIO.setRollerVelocity(-80);
        });
  }

  @Override
  public Command restExtended() {
    return this.run(
            () -> {
              pivotIO.setMotorPositionSetpoint(PIVOT_EXTENDED_POSITION);
              rollerIO.setRollerVoltage(0.0);
            })
        .until(atExtensionTrigger)
        .andThen(
            this.run(
                () -> {
                  pivotIO.setMotorVoltage(0);
                  rollerIO.setRollerVoltage(0);
                }));
  }

  @Override
  public Command restRetracted() {
    return this.run(
        () -> {
          pivotIO.setMotorPositionSetpoint(PIVOT_RETRACTED_POSITION);
          rollerIO.setRollerVoltage(0.0);
        });
  }

  @Override
  public Command runCurrentZeroing() {
    return Commands.sequence(
        this.run(() -> pivotIO.setMotorVoltage(-2)), // TODO: TUNE VOLTAGE
        Commands.waitUntil(() -> currentFilterValue > CURRENT_ZEROING_THRESHOLD),
        this.runOnce(() -> pivotIO.resetEncoder(PIVOT_MIN_POSITION)),
        Commands.print("Intake pivot zeroed"));
  }

  @Override
  public Command zeroPivotOffCancoder() {
    return this.runOnce(() -> pivotIO.resetEncoder(cancoderIOInputs.cancoderPositionRotations));
  }

  @Override
  public boolean beambreak() {
    // No beambreak lol
    return false;
  }

  @Override
  public Rotation2d getPosition() {
    return pivotIOInputs.position;
  }

  @Override
  public Rotation2d getPositionSetpoint() {
    return pivotIO.getSetpoint();
  }

  public boolean atExtension() {
    return MathUtil.isNear(getPositionSetpoint().getDegrees(), getPosition().getDegrees(), 10);
  }

  public static TalonFXConfiguration getPivotConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = 6;
    config.Feedback.RotorToSensorRatio = PIVOT_GEAR_RATIO;

    config.Feedback.SensorToMechanismRatio = 1;

    config.Slot0.kS = 0.05;
    config.Slot0.kV = 8.0; // Might suck\
    config.Slot0.kA = 0.0;
    config.Slot0.kG = 0.55;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.GravityArmPositionOffset = 0.0; // Maybe need this??
    config.Slot0.kP = 15.0;
    config.Slot0.kD = 0.3;

    config.CurrentLimits.StatorCurrentLimit = 45.0; // glup
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // TODO: TUNE
    config.MotionMagic.MotionMagicCruiseVelocity = .5;
    config.MotionMagic.MotionMagicAcceleration = 10;

    return config;
  }

  public static TalonFXConfiguration getRollerConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO

    config.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;

    config.Slot0.kS = 1; // 0.55127;
    config.Slot0.kV = 0.18; // 0.19756;
    config.Slot0.kA = 0; // 0.0074445;
    config.Slot0.kP = 0.55; // 0.017985;
    config.Slot0.kD = 0.0;

    // TODO: TUNE
    config.CurrentLimits.StatorCurrentLimit = 25.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }

  public static CANcoderConfiguration getCancoderConfig() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = 0.510498;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

    return config;
  }
}
