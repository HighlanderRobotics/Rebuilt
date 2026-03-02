package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
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
  public static final Rotation2d PIVOT_EXTENDED_POSITION = Rotation2d.kZero; // TODO
  public static final Rotation2d PIVOT_RETRACTED_POSITION = Rotation2d.kZero; // TODO
  public static final Rotation2d PIVOT_MIN_POSITION = Rotation2d.kZero; // TODO
  public static final Rotation2d PIVOT_MAX_POSITION = Rotation2d.kZero; // TODO
  public static final double CURRENT_ZEROING_THRESHOLD = 30.0; // TODO: TUNE
  public static final double ROLLER_GEAR_RATIO = 1.0; // TODO
  public static final double PIVOT_GEAR_RATIO = 1.0; // TODO

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

  private final SysIdRoutine rollerSysid;

  private final SysIdRoutine pivotSysid;

  public SlapdownSubsystem(PivotIO pivotIO, CANcoderIO cancoderIO, RollerIO rollerIO) {
    this.pivotIO = pivotIO;
    this.cancoderIO = cancoderIO;
    this.rollerIO = rollerIO;

    rollerSysid =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/Roller/Sysid State", state.toString())),
            new Mechanism((volts) -> rollerIO.setRollerVoltage(volts.in(Volt)), null, this));

    pivotSysid =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/Pivot/Sysid State", state.toString())),
            new Mechanism((volts) -> pivotIO.setMotorVoltage(volts.in(Volt)), null, this));
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
                      pivotIO.setMotorPositionSetpoint(PIVOT_EXTENDED_POSITION);
                      rollerIO.setRollerVelocity(10.0);
                    })
                .until(atExtensionTrigger),
            this.run(
                    () -> {
                      pivotIO.setMotorPositionSetpoint(
                          PIVOT_EXTENDED_POSITION.minus(Rotation2d.fromDegrees(30))); // TODO: TUNE
                      rollerIO.setRollerVelocity(10.0);
                    })
                .until(atExtensionTrigger))
        .repeatedly();
  }

  @Override
  public Command intake() {
    return this.run(
        () -> {
          pivotIO.setMotorPositionSetpoint(PIVOT_EXTENDED_POSITION);
          rollerIO.setRollerVelocity(40);
        });
  }

  @Override
  public Command restExtended() {
    return this.run(
        () -> {
          pivotIO.setMotorPositionSetpoint(PIVOT_EXTENDED_POSITION);
          rollerIO.setRollerVoltage(0.0);
        });
  }

  @Override
  public Command runCurrentZeroing() {
    return Commands.sequence(
        this.run(() -> pivotIO.setMotorVoltage(-2)), // TODO: TUNE VOLTAGE
        Commands.waitUntil(() -> currentFilterValue > CURRENT_ZEROING_THRESHOLD),
        this.runOnce(() -> pivotIO.resetEncoder(Rotation2d.kZero)),
        Commands.print("Intake pivot zeroed"));
  }

  @Override
  public Command runRollerSysid() {
    return Commands.sequence(
        rollerSysid.quasistatic(Direction.kForward),
        rollerSysid.quasistatic(Direction.kReverse),
        rollerSysid.dynamic(Direction.kForward),
        rollerSysid.dynamic(Direction.kReverse));
  }

  @Override
  public Command runPivotSysid() {
    return Commands.sequence(
        pivotSysid
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    pivotIOInputs.position.getDegrees()
                        > (PIVOT_MAX_POSITION.getDegrees() - 5) // Stop 5 degrees before hardstop
                ),
        pivotSysid
            .quasistatic(Direction.kReverse)
            .until(
                () -> pivotIOInputs.position.getDegrees() < (PIVOT_MIN_POSITION.getDegrees() + 5)),
        pivotSysid
            .dynamic(Direction.kForward)
            .until(
                () -> pivotIOInputs.position.getDegrees() > (PIVOT_MAX_POSITION.getDegrees() - 5)),
        pivotSysid
            .dynamic(Direction.kReverse)
            .until(
                () -> pivotIOInputs.position.getDegrees() < (PIVOT_MIN_POSITION.getDegrees() + 5)));
  }

  @Override
  public Command zeroRackOffCancoder() {
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
    return MathUtil.isNear(
        getPositionSetpoint().getDegrees(), getPosition().getDegrees(), 2); // TODO: TUNE TOLERANCE
  }

  public static TalonFXConfiguration getPivotConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO

    config.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.GravityArmPositionOffset = 0.0; // Maybe need this??
    config.Slot0.kP = 0.0;
    config.Slot0.kD = 0.0;

    // TODO: TUNE
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // TODO: TUNE
    config.MotionMagic.MotionMagicCruiseVelocity = 5;
    config.MotionMagic.MotionMagicAcceleration = 10;

    return config;
  }

  public static TalonFXConfiguration getRollerConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO

    config.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;

    config.Slot0.kS = 0.55127;
    config.Slot0.kV = 0.19756;
    config.Slot0.kA = 0.0074445;
    config.Slot0.kP = 0.017985;
    config.Slot0.kD = 0.0;

    // TODO: TUNE
    config.CurrentLimits.StatorCurrentLimit = 55.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }

  public static CANcoderConfiguration getCancoderConfig() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    // TODO: TUNE
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    config.MagnetSensor.MagnetOffset = 0.0;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

    return config;
  }
}
