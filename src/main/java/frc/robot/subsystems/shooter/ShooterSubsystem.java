// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Fixed shooter. !! ALPHA !! */
public class ShooterSubsystem extends SubsystemBase implements Shooter {
  public static double HOOD_GEAR_RATIO = 24.230769;
  public static Rotation2d HOOD_MAX_ROTATION = Rotation2d.fromDegrees(40);
  public static Rotation2d HOOD_MIN_ROTATION = Rotation2d.fromDegrees(2);

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

  /** Creates a new HoodSubsystem. */
  public ShooterSubsystem(HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;
  }

  @Override
  public Command testShoot() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(Rotation2d.fromDegrees(testDegrees.get()));
          flywheelIO.setMotionProfiledFlywheelVelocity(testVelocity.get());
        });
  }

  @Override
  public Command shoot(Supplier<Pose2d> robotPoseSupplier) {
    return this.run(
        () -> {
          ShotData shotData =
              AutoAim.HUB_SHOT_TREE.get(AutoAim.distanceToHub(robotPoseSupplier.get()));
          hoodIO.setHoodPosition(shotData.hoodAngle());
          flywheelIO.setMotionProfiledFlywheelVelocity(shotData.flywheelVelocityRotPerSec());
        });
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
        });
  }

  @Override
  public Command rest() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(HOOD_MIN_ROTATION); // TODO: TUNE TUCKED POSITION IF NEEDED
          flywheelIO.setFlywheelVoltage(0.0);
        });
  }

  @Override
  public Command spit() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(Rotation2d.kZero);
          flywheelIO.setMotionProfiledFlywheelVelocity(20);
        }); // TODO: TUNE HOOD POS AND FLYWHEEL VELOCITY
  }

  public Command setHoodPositionCommand(Supplier<Rotation2d> hoodPosition) {
    return this.run(() -> hoodIO.setHoodPosition(hoodPosition.get()));
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);

    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
  }

  public Command runHoodSysid() {
    return Commands.sequence(
        hoodSysid
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    hoodInputs.hoodPositionRotations.getDegrees()
                        > (HOOD_MAX_ROTATION.getDegrees() - 5)),
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

  public Command runFlywheelSysid() {
    return Commands.sequence(
        flywheelSysid.quasistatic(Direction.kForward),
        flywheelSysid.quasistatic(Direction.kReverse),
        flywheelSysid.dynamic(Direction.kForward),
        flywheelSysid.dynamic(Direction.kReverse));
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
        hoodInputs.hoodPositionRotations.getDegrees(), hoodIO.getHoodSetpoint().getDegrees(), 1);
  }

  @Override
  public Command zeroHood() {
    return this.runOnce(() -> hoodIO.resetEncoder(HOOD_MIN_ROTATION));
  }
}
