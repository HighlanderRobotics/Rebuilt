package frc.robot.utils.autoaim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import org.littletonrobotics.junction.Logger;

public class AutoAim {

  public static final InterpolatingShotTree HUB_SHOT_TREE = new InterpolatingShotTree();

  // If we need other shot trees (i.e. for feeding) we can put them here

  public static boolean isInTolerance(
      Pose2d current,
      Pose2d target,
      double translationalToleranceMeters,
      double angularToleranceRadians) {
    Transform2d diff = current.minus(target);
    return MathUtil.isNear(0.0, Math.hypot(diff.getX(), diff.getY()), translationalToleranceMeters)
        && MathUtil.isNear(
            target.getRotation().getRadians(),
            current.getRotation().getRadians(),
            angularToleranceRadians);
  }

  public static final InterpolatingShotTree shotMap = new InterpolatingShotTree();

  static {
    // TODO find actual numbers
    shotMap.put(1.0, new ShotData(Rotation2d.kZero, 0, 0));
  }

  public static Pose3d getVirtualSOTMTarget(
      Pose3d target, ChassisSpeeds fieldRelativeSpeeds, double shotTime) {
    // velocity times shot time is how translated it is
    Pose3d vtarget =
        target.transformBy(
            new Transform3d(
                    fieldRelativeSpeeds.vxMetersPerSecond * shotTime,
                    fieldRelativeSpeeds.vyMetersPerSecond * shotTime,
                    0,
                    new Rotation3d())
                .inverse());
    Logger.recordOutput("Autoaim/Virtual Target", vtarget);
    return vtarget;
  }

  public static Rotation2d getTurretSOTMAzimuth(
      Pose3d goal, Pose3d robot, ChassisSpeeds fieldRelativeSpeeds) {
    // V_ball-ground = V_ball-robot + V_robot-ground (relative motion)
    // if we want the ball to go straight towards the goal,
    // the V_ball-robot vector needs to cancel out with the V_robot-ground vector to "offset" the
    // velocity it already has
    // the ball exits the shooter with velocity v at an angle theta (just assume it's the correct
    // velocity and angle)
    // the magnitude of the V_ball-ground vector (or |V_ball-ground|) is v * cos (theta)
    double fuelHorizVelocity =
        AutoAim.shotMap.calculateShot(goal, robot).flywheelVelocityRotPerSec()
            * AutoAim.shotMap.calculateShot(goal, robot).hoodAngle().getCos();
    // let phi be the azimuth
    // phi = arcsin(-V_robot-ground / |V_ball-ground|)
    double phi = Math.asin((-1) * fieldRelativeSpeeds.vyMetersPerSecond / fuelHorizVelocity);
    return Rotation2d.fromRadians(phi);
  }

  public static Pair<Rotation2d, Double> getShooterState(
      Pose3d goal, Pose3d robot, ChassisSpeeds fieldRelativeSpeeds) {
    Rotation2d hoodTarget = AutoAim.shotMap.calculateShot(goal, robot).hoodAngle();
    double shooterVelocity = AutoAim.shotMap.calculateShot(goal, robot).flywheelVelocityRotPerSec();
    return Pair.of(hoodTarget, shooterVelocity);
  }
}
