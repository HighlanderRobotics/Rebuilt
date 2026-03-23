// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import frc.robot.utils.autoaim.NewAutoAim.ShotParams;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Fixed shooter. !! ALPHA !! */
public class ShooterSubsystem extends SubsystemBase implements Shooter {
  public static double HOOD_GEAR_RATIO = 24.230769;

  public static Rotation2d HOOD_MAX_ROTATION = Rotation2d.fromDegrees(40);
  public static Rotation2d HOOD_MIN_ROTATION = Rotation2d.fromDegrees(2);
  public static double CURRENT_ZERO_THRESHOLD = 30.0;

  public static double FLYWHEEL_GEAR_RATIO = 28.0 / 24.0;

  public static double FLYWHEEL_VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND = 5.0;

  HoodIO hoodIO;
  HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  FlywheelIO flywheelIO;
  FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private SysIdRoutine hoodSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Shooter/Hood/SysID State", state.toString())),
          new Mechanism((voltage) -> hoodIO.setHoodVoltage(voltage.in(Volts)), null, this));

  private SysIdRoutine flywheelSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Shooter/Flywheel/SysID State", state.toString())),
          new Mechanism((voltage) -> flywheelIO.setFlywheelVoltage(voltage.in(Volts)), null, this));

  private LoggedTunableNumber testDegrees = new LoggedTunableNumber("Shooter/Test Degrees", 10.0);
  private LoggedTunableNumber testVelocity = new LoggedTunableNumber("Shooter/Test Velocity", 30.0);

  private LinearFilter currentFilter = LinearFilter.movingAverage(10);

  @AutoLogOutput(key = "Shooter/Hood/Setpoint")
  public Rotation2d hoodSetpoint = Rotation2d.kZero;

  @AutoLogOutput(key = "Shooter/Hood/Current Filter Value")
  private double currentFilterValue = 0.0;

  /** Creates a new HoodSubsystem. */
  public ShooterSubsystem(HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;
  }

  public Command score(Supplier<ShotParams> shotParamsSupplier) {
    return this.run(
        () -> {
          hoodSetpoint = shotParamsSupplier.get().shotData().hoodAngle();
          hoodIO.setHoodPosition(shotParamsSupplier.get().shotData().hoodAngle());
          flywheelIO.setMotionProfiledFlywheelVelocity(
              shotParamsSupplier.get().shotData().flywheelVelocityRotPerSec());
        });
  }

  @Override
  public Command feed(
      Supplier<ShotParams> shotParamsSupplier,
      Supplier<Pose2d> feedTarget,
      Supplier<Pose2d> robotPoseSupplier) {
    return this.run(
        () -> {
          ShotData shotData =
              AutoAim.FEED_SHOT_TREE.get(
                  robotPoseSupplier
                      .get()
                      .getTranslation()
                      .getDistance(feedTarget.get().getTranslation()));
          hoodIO.setHoodPosition(shotData.hoodAngle());
          flywheelIO.setMotionProfiledFlywheelVelocity(shotData.flywheelVelocityRotPerSec());
        });
  }

  @Override
  public Command rest(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      BooleanSupplier canScore,
      Supplier<Pose2d> feedTarget) {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(HOOD_MIN_ROTATION);
          flywheelIO.setFlywheelVoltage(0.0);
          // flywheelIO.setMotionProfiledFlywheelVelocity(30);
        });
  }

  @Override
  public Command spit() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(HOOD_MIN_ROTATION);
          flywheelIO.setMotionProfiledFlywheelVelocity(20);
        }); // TODO: TUNE HOOD POS AND FLYWHEEL VELOCITY
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);

    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);

    currentFilterValue = currentFilter.calculate(hoodInputs.hoodStatorCurrentAmps);
  }

  @Override
  @AutoLogOutput(key = "Shooter/At Vel Setpoint")
  public boolean atFlywheelVelocitySetpoint() {
    return MathUtil.isNear(
        flywheelInputs.flywheelLeaderVelocityRotationsPerSecond,
        flywheelIO.getSetpointRotPerSec(),
        FLYWHEEL_VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND);
  }

  @Override
  @AutoLogOutput(key = "Shooter/Hood/At Setpoint")
  public boolean atHoodSetpoint() {
    return MathUtil.isNear(
        hoodInputs.hoodPositionRotations.getDegrees(), hoodIO.getHoodSetpoint().getDegrees(), 5);
  }

  @Override
  public Command zeroHood() {
    return this.runOnce(() -> hoodIO.resetEncoder(HOOD_MIN_ROTATION));
  }

  @Override
  public Command runHoodCurrentZeroing() {
    return this.run(() -> hoodIO.setHoodVoltage(-3.0))
        .until(
            new Trigger(() -> Math.abs(currentFilterValue) > CURRENT_ZERO_THRESHOLD).debounce(0.25))
        .andThen(Commands.parallel(Commands.print("Hood Zeroed"), zeroHood()));
  }

  @Override
  public Rotation2d getHoodSetpoint() {
    return hoodSetpoint;
  }

  @Override
  public Rotation2d getHoodPosition() {
    return hoodInputs.hoodPositionRotations;
  }

  // Only for comp turret
  @Override
  public boolean atTurretSetpoint() {
    return false;
  }

  public static TalonFXConfiguration getFlywheelConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ShooterSubsystem.FLYWHEEL_GEAR_RATIO;

    config.Slot0.kS = 0.43477;
    config.Slot0.kV = 0.144;
    config.Slot0.kA = 0.016433;
    config.Slot0.kP = 0.37;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.StatorCurrentLimit = 70.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;

    config.MotionMagic.MotionMagicAcceleration = 100.0;

    return config;
  }

  public static TalonFXConfiguration getHoodConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ShooterSubsystem.HOOD_GEAR_RATIO;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kS = 0.055;
    config.Slot0.kG = 0.445;
    config.Slot0.kV = 1.45;
    config.Slot0.kP = 35;
    config.Slot0.kD = 0.25;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;

    return config;
  }
}
