package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.common.collect.ImmutableSet;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.module.Module.ModuleConstants;
import frc.robot.subsystems.swerve.odometry.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.odometry.PhoenixOdometryThread.Registration;
import frc.robot.subsystems.swerve.odometry.PhoenixOdometryThread.SignalType;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class ModuleIOReal {

  @AutoLog
  public static class ModuleIOInputs {
    // For the drive motor
    // The conversions from rotations to meters are done in the SensorToMechanismRatio config for
    // the motors
    public boolean driveConnected = false;
    public double driveVelocityMetersPerSec = 0.0;
    public double drivePositionMeters = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveTempC = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveSupplyCurrentAmps = 0.0;

    // For the x motor
    public boolean turnConnected = false;
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnTempC = 0.0;
    public double turnStatorCurrentAmps = 0.0;
    public double turnSupplyCurrentAmps = 0.0;

    public boolean cancoderConnected = false;
    public Rotation2d cancoderAbsolutePosition = new Rotation2d();
  }

  private final ModuleConstants constants;

  // Hardware
  protected final TalonFX driveTalon;
  protected final TalonFX turnTalon;
  protected final CANcoder cancoder;

  // Status signals
  // For drive
  private final BaseStatusSignal drivePosition;
  private final BaseStatusSignal driveVelocity;
  private final BaseStatusSignal driveTemp;
  private final BaseStatusSignal driveSupplyCurrent;
  private final BaseStatusSignal driveStatorCurrent;
  private final BaseStatusSignal driveAppliedVolts;

  // For turn
  private final BaseStatusSignal cancoderAbsolutePosition;
  private final BaseStatusSignal turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final BaseStatusSignal turnTemp;
  private final BaseStatusSignal turnSupplyCurrent;
  private final BaseStatusSignal turnStatorCurrent;
  private final BaseStatusSignal turnAppliedVolts;

  // Control modes
  private final VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut turnVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC driveVelocityControl =
      new VelocityTorqueCurrentFOC(0.0).withSlot(0);
  private final MotionMagicVoltage turnPID = new MotionMagicVoltage(0.0);

  public ModuleIOReal(ModuleConstants moduleConstants, CANBus canbus) {
    this.constants = moduleConstants;

    // Initialize hardware
    driveTalon = new TalonFX(constants.driveID(), canbus);
    turnTalon = new TalonFX(constants.turnID(), canbus);
    cancoder = new CANcoder(constants.cancoderID(), canbus);

    // Configure hardware
    driveTalon.getConfigurator().apply(SwerveSubsystem.SWERVE_CONSTANTS.getDriveConfig());
    turnTalon
        .getConfigurator()
        .apply(SwerveSubsystem.SWERVE_CONSTANTS.getTurnConfig(constants.cancoderID()));

    cancoder
        .getConfigurator()
        .apply(SwerveSubsystem.SWERVE_CONSTANTS.getCancoderConfig(constants.cancoderOffset()));

    // Initialize status signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveTemp = driveTalon.getDeviceTemp();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveStatorCurrent = driveTalon.getStatorCurrent();
    driveAppliedVolts = driveTalon.getMotorVoltage();

    cancoderAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnTemp = turnTalon.getDeviceTemp();
    turnSupplyCurrent = turnTalon.getSupplyCurrent();
    turnStatorCurrent = turnTalon.getStatorCurrent();
    turnAppliedVolts = turnTalon.getMotorVoltage();

    // Update the signals not updated by the odo thread at 50 hz
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveTemp,
        driveAppliedVolts,
        driveStatorCurrent,
        driveSupplyCurrent,
        cancoderAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnStatorCurrent,
        turnSupplyCurrent);
    // Register the signals to the odo thread
    PhoenixOdometryThread.getInstance()
        .registerSignals(
            new Registration(
                driveTalon,
                Optional.of(moduleConstants),
                SignalType.DRIVE,
                ImmutableSet.of(drivePosition)),
            new Registration(
                turnTalon,
                Optional.of(moduleConstants),
                SignalType.TURN,
                ImmutableSet.of(turnPosition)));
    // Still need to tell the motors to update these faster, even though we've registered them as
    // signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        PhoenixOdometryThread.ODOMETRY_FREQUENCY_HZ, drivePosition, turnPosition);

    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
  }

  public void updateInputs(ModuleIOInputs inputs) {
    Logger.recordOutput(
        "Module" + constants.prefix() + "refresh status code",
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveStatorCurrent,
            driveSupplyCurrent,
            driveTemp,
            cancoderAbsolutePosition,
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnStatorCurrent,
            turnSupplyCurrent,
            turnTemp));

    inputs.driveConnected =
        BaseStatusSignal.isAllGood(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveStatorCurrent,
            driveSupplyCurrent,
            driveTemp);
    inputs.drivePositionMeters = drivePosition.getValueAsDouble();
    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveTempC = driveTemp.getValueAsDouble();
    inputs.driveStatorCurrentAmps = driveStatorCurrent.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();

    inputs.turnConnected =
        BaseStatusSignal.isAllGood(
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnStatorCurrent,
            turnSupplyCurrent,
            turnTemp);
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = turnVelocity.getValue().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnTempC = turnTemp.getValueAsDouble();
    inputs.turnStatorCurrentAmps = turnStatorCurrent.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();

    inputs.cancoderConnected = BaseStatusSignal.isAllGood(cancoderAbsolutePosition);
    inputs.cancoderAbsolutePosition =
        Rotation2d.fromRotations(cancoderAbsolutePosition.getValueAsDouble());
  }

  public void setDriveVoltage(double volts, boolean withFoc) {
    driveTalon.setControl(driveVoltage.withOutput(volts).withEnableFOC(withFoc));
  }

  public void setDriveVoltage(double volts) {
    setDriveVoltage(volts, true);
  }

  public void setDriveVelocitySetpoint(double setpointMetersPerSecond) {
    driveTalon.setControl(driveVelocityControl.withVelocity(setpointMetersPerSecond));
  }

  public void setTurnVoltage(double volts) {
    turnTalon.setControl(turnVoltage.withOutput(volts));
  }

  public void setTurnPositionSetpoint(Rotation2d setpoint) {
    turnTalon.setControl(turnPID.withPosition(setpoint.getRotations()));
  }

  public ModuleConstants getModuleConstants() {
    return constants;
  }
}
