// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.InterpolatingShotTree;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  protected final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  protected final ShooterIOReal shooterIO;
  protected final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  protected final PivotIOReal pivotIO;
  protected final PivotIOInputsAutoLogged hoodInputs = new PivotIOInputsAutoLogged();
  protected final PivotIOReal hoodIO;
  InterpolatingShotTree tree;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(ShooterIOReal shooterIO, PivotIOReal pivotIO, PivotIOReal hoodIO) {
    this.shooterIO = shooterIO;
    this.pivotIO = pivotIO;
    this.hoodIO = hoodIO;
  }

  public Command shootCommand(
      Supplier<Rotation2d> pivotTarget,
      DoubleSupplier rollerVelocity,
      Supplier<Rotation2d> hoodTarget) {
    return this.run(
        () -> {
          Logger.recordOutput("Pivot Setpoint", pivotTarget.get());
          pivotIO.setMotorPosition(pivotTarget.get());
          Logger.recordOutput("Shooter Velocity", rollerVelocity.getAsDouble());
          shooterIO.setRollerVelocity(rollerVelocity.getAsDouble());
          Logger.recordOutput("Hood Setpoint", hoodTarget.get());
          hoodIO.setMotorPosition(hoodTarget.get());
        });
  }

  public Command shootCommand(Supplier<Rotation2d> pivotTarget, Supplier<ShotData> shotData) {
    return shootCommand(
        pivotTarget,
        () -> shotData.get().flywheelVelocityRotPerSec(),
        () -> shotData.get().hoodRotation());
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

  public Pose3d getPose3d(Supplier<Pose3d> robot3dposeSupplier, Supplier<Pose3d> targetSupplier) {
    Transform3d robotToTurret = new Transform3d(0, 0, 0.3, Rotation3d.kZero);
    Pose3d turretPose = robot3dposeSupplier.get().transformBy(robotToTurret);
    Transform3d turretToHub = targetSupplier.get().minus(turretPose);
    Rotation2d pivotTarget =
        Rotation2d.fromRadians(Math.atan2(turretToHub.getY(), turretToHub.getX()));
    Rotation2d hoodTarget =
        AutoAim.shotMap
            .calculateShot(targetSupplier.get(), robot3dposeSupplier.get())
            .hoodRotation();
    return new Pose3d(
        robot3dposeSupplier.get().getTranslation().plus(new Translation3d(0, 0, 0.3)),
        robot3dposeSupplier
            .get()
            .getRotation()
            .rotateBy(new Rotation3d(0, 0, pivotTarget.getRadians()))
            .rotateBy(new Rotation3d(0, (-1) * hoodTarget.getRadians(), 0)));
  }
}
