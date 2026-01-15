// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  protected final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  protected final ShooterIOReal shooterIO;
  protected final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  protected final PivotIOReal pivotIO;
  protected final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  protected final HoodIOReal hoodIO;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(ShooterIOReal shooterIO, PivotIOReal pivotIO, HoodIOReal hoodIO) {
    this.shooterIO = shooterIO;
    this.pivotIO = pivotIO;
    this.hoodIO = hoodIO;
  }

  public Command runStateCommand(
      Supplier<Rotation2d> pivotTarget,
      DoubleSupplier rollerVoltage,
      Supplier<Rotation2d> hoodTarget) {
    return this.run(
        () -> {
          Logger.recordOutput("Pivot Setpoint", pivotTarget.get());
          pivotIO.setMotorPosition(pivotTarget.get());
          Logger.recordOutput("Shooter Voltage", rollerVoltage.getAsDouble());
          shooterIO.setRollerVoltage(rollerVoltage.getAsDouble());
          Logger.recordOutput("Hood Setpoint", hoodTarget.get());
          hoodIO.setHoodPosition(hoodTarget.get());
        });
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Pivot", pivotInputs);
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Hood", hoodInputs);
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);
  }

  public Pose3d getPose3d(Supplier<Pose3d> robot3dposeSupplier) {
    Transform3d robotToTurret = new Transform3d(0, 0, 0.3, Rotation3d.kZero);
    Pose3d hubPose = new Pose3d(4.6, 4.03, Units.inchesToMeters(72), Rotation3d.kZero);
    Logger.recordOutput("Hub Pose", hubPose);
    Pose3d turretPose = robot3dposeSupplier.get().transformBy(robotToTurret);
    Transform3d turretToHub = hubPose.minus(turretPose);
    Rotation2d pivotTarget =
        Rotation2d.fromRadians(Math.atan2(turretToHub.getY(), turretToHub.getX()));
    double distanceToHub = Math.hypot(turretToHub.getX(), turretToHub.getY());
    // magic function that calculates arctangent of z and the distance from turret to hub then the
    // parabola because :sparkle: kinematics
    Rotation2d hoodTarget = Rotation2d.fromRadians(Math.atan2(turretToHub.getZ(), distanceToHub));
    return new Pose3d(
        robot3dposeSupplier.get().getTranslation().plus(new Translation3d(0, 0, 0.3)),
        robot3dposeSupplier
            .get()
            .getRotation()
            .rotateBy(new Rotation3d(0, 0, pivotTarget.getRadians()))
            .rotateBy(new Rotation3d(0, (-1) * hoodTarget.getRadians(), 0)));
  }
}
