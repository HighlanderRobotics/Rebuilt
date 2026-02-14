// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Superstructure;
import frc.robot.components.cancoder.CANcoderIO;
import frc.robot.components.cancoder.CANcoderIOInputsAutoLogged;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.FeedTargets;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Pivoting hooded shooter (turret). !! COMP !! */
public class TurretSubsystem extends SubsystemBase implements Shooter {

  /** Creates a new TurretSubsystem. */
  public static double HOOD_GEAR_RATIO = 58.96875;

  public static double FLYWHEEL_GEAR_RATIO = 0.84615384615;

  public static Rotation2d HOOD_MAX_ROTATION = Rotation2d.fromDegrees(73);
  public static Rotation2d HOOD_MIN_ROTATION = Rotation2d.fromDegrees(23.16);
  public static double CURRENT_ZERO_THRESHOLD = 30.0;

  public static Rotation2d TURRET_MIN_ROTATIONS = Rotation2d.fromRotations(-0.719536);
  public static Rotation2d TURRET_MAX_ROTATIONS = Rotation2d.fromRotations(0.011378);

  public static double FLYWHEEL_VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND = 5.0;
  double currentFilterValue = 0.0;

  private CANcoderIO cancoder24t;
  private CANcoderIO cancoder26t;
  private CANcoderIOInputsAutoLogged cancoder24tInputs = new CANcoderIOInputsAutoLogged();
  private CANcoderIOInputsAutoLogged cancoder26tInputs = new CANcoderIOInputsAutoLogged();

  private HoodIO hoodIO;
  private HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private FlywheelIO flywheelIO;
  private FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private TurretIO turretIO;
  private TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  private LinearFilter currentFilter = LinearFilter.movingAverage(10);

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

  private SysIdRoutine turretSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Shooter/Turret/SysID State", state.toString())),
          new Mechanism((voltage) -> turretIO.setVoltage(voltage.in(Volts)), null, this));

  // TODO actually correctly set this everywhere
  private Rotation2d hoodSetpoint = Rotation2d.kZero;
  private double flywheelVelSetpoint = 0.0;

  private LoggedTunableNumber testDegrees =
      new LoggedTunableNumber("Shooter/Test Hood Degrees", 30.0);
  private LoggedTunableNumber testVelocity = new LoggedTunableNumber("Shooter/Test Velocity", 30.0);

  private static final Alert cancoder24tDisconnectedAlert =
      new Alert("24T Cancoder disconnected!", AlertType.kError);
  private static final Alert cancoder26tDisconnectedAlert =
      new Alert("26T Cancoder disconnected!", AlertType.kError);

  public TurretSubsystem(
      FlywheelIO flywheelIO,
      HoodIO hoodIO,
      TurretIO turretIO,
      CANcoderIO cancoder24t,
      CANcoderIO cancoder26t) {
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
    this.turretIO = turretIO;
    this.cancoder24t = cancoder24t;
    this.cancoder26t = cancoder26t;

    // Starting positions
    this.cancoder24t.updateInputs(cancoder24tInputs);
    this.cancoder26t.updateInputs(cancoder26tInputs);

    Logger.processInputs("Shooter/Turret Cancoder24t", cancoder24tInputs);
    Logger.processInputs("Shooter/Turret Cancoder26t", cancoder26tInputs);

    // Calculate the start angle based on the two encoders
    Rotation2d startAngle = getCalculatedTurretRotations();
    Logger.recordOutput("Shooter/Turret/Starting Angle", startAngle);
    // Set the sensor position to the start angle
    turretIO.resetTurretEncoder(startAngle);
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("Shooter/Turret", turretInputs);
    cancoder24t.updateInputs(cancoder24tInputs);
    Logger.processInputs("Shooter/Turret Cancoder24t", cancoder24tInputs);
    cancoder26t.updateInputs(cancoder26tInputs);
    Logger.processInputs("Shooter/Turret Cancoder26t", cancoder26tInputs);
    currentFilterValue = currentFilter.calculate(hoodInputs.hoodStatorCurrentAmps);

    cancoder24tDisconnectedAlert.set(cancoder24tInputs.connected);
    cancoder26tDisconnectedAlert.set(cancoder26tInputs.connected);
  }

  public static CANcoderConfiguration getCancoder24tConfigs() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset =
        -0.304199 - TurretIO.CANCODER_24T_TO_TURRET_GEAR_RATIO / 0.1; // 0.696;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;

    return config;
  }

  public static CANcoderConfiguration getCancoder26tConfigs() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset =
        -0.371 - TurretIO.CANCODER_26T_TO_TURRET_GEAR_RATIO / 0.1; // 0.623;x
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;

    return config;
  }

  @Override
  public Command feed(Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> feedTarget) {
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
          turretIO.setTurretPosition(
              AutoAim.getTurretTargetRotation(
                  FeedTargets.getFeedTarget(Superstructure.getFeedTarget())
                      .getPose()
                      .getTranslation(),
                  robotPoseSupplier.get()));
        });
  }

  @Override
  public Command rest() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(HOOD_MIN_ROTATION); // TODO: TUNE TUCKED POSITION IF NEEDED
          flywheelIO.setFlywheelVoltage(0.0);
          // turretIO.setTurretPosition(TurretIO.TURRET_MIN_ROTATIONS);
        });
  }

  @Override
  public Command spit() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(HOOD_MIN_ROTATION);
          flywheelIO.setMotionProfiledFlywheelVelocity(20);
          turretIO.setTurretPosition(TURRET_MIN_ROTATIONS);
        }); // TODO: TUNE HOOD POS AND FLYWHEEL VELOCITY
  }

  @Override
  @AutoLogOutput(key = "Shooter/Turret/Turret Calculated Rotation")
  public Rotation2d getCalculatedTurretRotations() {
    // give valaues between 0 and 1
    Rotation2d cancoder24t = cancoder24tInputs.cancoderPositionRotations;
    Rotation2d cancoder26t = cancoder26tInputs.cancoderPositionRotations;

    // if can one is bigger than can 2 its simply can1-can2
    // otherwise can1 + 1 - can2 because we want how much behind can1 it is
    // double diffRotations = (cancoder24t.getRotations() - cancoder26t.getRotations()) % 1;
    double diffRotations =
        // MathUtil.applyDeadband(cancoder24t.getRotations() - cancoder26t.getRotations(), 0.01) > 0
        cancoder24t.getRotations() > cancoder26t.getRotations()
            ? cancoder24t.getRotations() - cancoder26t.getRotations()
            : cancoder24t.getRotations() + 1 - cancoder26t.getRotations();
    Logger.recordOutput(
        "Turret/Uncorrected diff rotations",
        cancoder24t.getRotations() - cancoder26t.getRotations());
    Logger.recordOutput("Turret/Diff Rotations", diffRotations);
    // TODO java seems to not know how mod works
    // keeping track of how many total rots can1 is doing using the diff with can2
    // 26/2 because gear difference of 2
    double absoluteRotationsCan1 = diffRotations * (13.0);
    Logger.recordOutput("Turret/Absolute Rotations Can 24t", absoluteRotationsCan1);

    // turret maxes out at less then 1 rotation which is like 11 can1 rotations anyways and it
    // should work up to there
    // multiply abs can1 rots by the gear ratio
    double turretRotations = absoluteRotationsCan1 * TurretIO.CANCODER_24T_TO_TURRET_GEAR_RATIO;
    turretRotations = MathUtil.inputModulus(turretRotations + 0.1, 0, 1);

    // offset such that the shooter facing forward is 0
    return Rotation2d.fromRotations(turretRotations - (325.58 / 360.0));
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
        hoodInputs.hoodPositionRotations.getDegrees(), getHoodSetpoint().getDegrees(), 1);
  }

  @Override
  @AutoLogOutput(key = "Shooter/Turret/At Setpoint")
  public boolean isFacingTarget() {
    return MathUtil.isNear(
        getCalculatedTurretRotations().getDegrees(), getTurretSetpoint().getDegrees(), 2);
  }

  @Override
  @AutoLogOutput(key = "Shooter/Hood/Setpoint")
  public Rotation2d getHoodSetpoint() {
    return hoodIO.getHoodSetpoint();
  }

  @AutoLogOutput(key = "Shooter/Turret/Setpoint")
  public Rotation2d getTurretSetpoint() {
    return turretIO.getTurretSetpoint();
  }

  @AutoLogOutput(key = "Shooter/Turret/Cancoder 24t position")
  public Rotation2d getTurretCancoder24tPosition() {
    return cancoder24tInputs.cancoderPositionRotations;
  }

  @AutoLogOutput(key = "Shooter/Turret/Cancoder 26t position")
  public Rotation2d getTurretCancoder26tPosition() {
    return cancoder26tInputs.cancoderPositionRotations;
  }

  @Override
  public Command zeroHood() {
    return this.runOnce(() -> hoodIO.resetEncoder(HOOD_MIN_ROTATION));
  }

  public Command runCurrentZeroing() {
    return this.run(() -> hoodIO.setHoodVoltage(-3.0))
        .until(
            new Trigger(() -> Math.abs(currentFilterValue) > CURRENT_ZERO_THRESHOLD).debounce(0.25))
        .andThen(Commands.parallel(Commands.print("Hood Zeroed"), zeroHood()));
  }

  @Override
  public Command testShoot() {
    return this.run(
        () -> {
          hoodSetpoint = Rotation2d.fromDegrees(testDegrees.get());
          hoodIO.setHoodPosition(Rotation2d.fromDegrees(testDegrees.get()));
          flywheelVelSetpoint = testVelocity.get();
          flywheelIO.setMotionProfiledFlywheelVelocity(testVelocity.get());
          // turretIO.setTurretPosition(TurretIO.TURRET_MIN_ROTATIONS);
        });
  }

  public Command score(Supplier<Pose2d> robotPoseSupplier, Supplier<ShotData> shotDataSupplier) {
    return this.run(
        () -> {
          hoodSetpoint = shotDataSupplier.get().hoodAngle();
          hoodIO.setHoodPosition(shotDataSupplier.get().hoodAngle());
          flywheelIO.setMotionProfiledFlywheelVelocity(
              shotDataSupplier.get().flywheelVelocityRotPerSec());
          turretIO.setTurretPosition(
              AutoAim.getTurretTargetRotation(
                  FieldUtils.getCurrentHubTranslation(), robotPoseSupplier.get()));
        });
  }

  @Override
  public Command runHoodSysid() {
    return Commands.sequence(
        hoodSysid
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    hoodInputs.hoodPositionRotations.getDegrees()
                        > (HOOD_MAX_ROTATION.getDegrees() - 5)), // Stop before endstop
        hoodSysid
            .quasistatic(Direction.kReverse)
            .until(
                () ->
                    hoodInputs.hoodPositionRotations.getDegrees()
                        < (HOOD_MIN_ROTATION.getDegrees() + 5)),
        hoodSysid
            .dynamic(Direction.kForward)
            .until(
                () ->
                    hoodInputs.hoodPositionRotations.getDegrees()
                        > (HOOD_MAX_ROTATION.getDegrees() - 5)),
        hoodSysid
            .dynamic(Direction.kReverse)
            .until(
                () ->
                    hoodInputs.hoodPositionRotations.getDegrees()
                        < (HOOD_MIN_ROTATION.getDegrees() + 5)));
  }

  @Override
  public Command runFlywheelSysid() {
    return Commands.sequence(
        flywheelSysid.quasistatic(Direction.kForward),
        flywheelSysid.quasistatic(Direction.kReverse),
        flywheelSysid.dynamic(Direction.kForward),
        flywheelSysid.dynamic(Direction.kReverse));
  }

  @Override
  public Command runTurretSysid() {
    return Commands.sequence(
        turretSysid
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    turretInputs.positionRotations.getDegrees()
                        > (TURRET_MAX_ROTATIONS.getDegrees() - 5)), // Stop before endstop
        turretSysid
            .quasistatic(Direction.kReverse)
            .until(
                () ->
                    turretInputs.positionRotations.getDegrees()
                        < (TURRET_MIN_ROTATIONS.getDegrees() + 5)),
        turretSysid
            .dynamic(Direction.kForward)
            .until(
                () ->
                    turretInputs.positionRotations.getDegrees()
                        > (TURRET_MAX_ROTATIONS.getDegrees() - 5)),
        turretSysid
            .dynamic(Direction.kReverse)
            .until(
                () ->
                    turretInputs.positionRotations.getDegrees()
                        < (TURRET_MIN_ROTATIONS.getDegrees() + 5)));
  }

  // public boolean isFacingTarget() {
  //   switch (Superstructure.getShotTarget()) { // ugh maybe this should be in robot.java
  //     case SCORE:
  //       return isFacingHub();
  //     case FEED:
  //       return isFacingFeedTarget();
  //     default:
  //       return false;
  //   }
  // }

  // public boolean isFacingHub() {
  //   Rotation2d target = AutoAim.getVirtualHubYaw(getVelocityFieldRelative(), getPose());
  //   return MathUtil.isNear(
  //       target.getRadians(), getPose().getRotation().getRadians(), 0.174533); // 10 degrees
  // }

  // public boolean isFacingFeedTarget() {
  //   Translation2d feedTarget =
  //       FeedTargets.getFeedTarget(Superstructure.getFeedTarget()).getPose().getTranslation();
  //   Rotation2d target = AutoAim.getTargetRotation(feedTarget, getPose());
  //   return MathUtil.isNear(
  //       target.getRadians(), getPose().getRotation().getRadians(), 0.174533); // 10 degrees
  // }

  @Override
  public Command resetTurretToPosition(Rotation2d rot) {
    return this.runOnce(() -> turretIO.resetTurretEncoder(getCalculatedTurretRotations()));
  }

  /** sets the motor encoder to the position calculated from the encoders */
  public Command resetTurretToCalculatedPosition() {
    return resetTurretToPosition(getCalculatedTurretRotations());
  }
}
