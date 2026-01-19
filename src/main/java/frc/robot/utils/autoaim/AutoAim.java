package frc.robot.utils.autoaim;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;

public class AutoAim {

  public static final InterpolatingShotTree HUB_SHOT_TREE = new InterpolatingShotTree();

  // If we need other shot trees (i.e. for feeding) we can put them here

  static { // For hub shot tree
    // TODO: ADD SHOTS TO HUB SHOT HERE
    HUB_SHOT_TREE.put(
        1.0, new ShotData(Rotation2d.kCW_90deg, 10, 0.5)); // Placeholder to prevent crashes
  }

  // Ig we'll see if we need more than 1 feed shot tree
  public static final InterpolatingShotTree FEED_SHOT_TREE = new InterpolatingShotTree();

  static { // For feed shot tree
    // TODO: POPULATE
    FEED_SHOT_TREE.put(
        1.0, new ShotData(Rotation2d.kCW_90deg, 10, 0.5)); // Placeholder to prevent crashes
  }

  // TODO: SOTM

  public static double distanceToHub(Pose2d pose) {
    return pose.getTranslation().getDistance(FieldUtils.getCurrentHubTranslation());
  }

  public static Rotation2d getSOTMYaw(Pose2d robot, ChassisSpeeds fieldRelativeSpeeds) {
    // V_ball-ground = V_ball-robot + V_robot-ground (relative motion)
    // if we want the ball to go straight towards the goal,
    // the V_ball-robot vector needs to cancel out with the V_robot-ground vector to "offset" the
    // velocity it already has
    // the ball exits the shooter with velocity v at an angle theta (just assume it's the correct
    // velocity and angle)
    // the magnitude of the V_ball-ground vector (or |V_ball-ground|) is v * cos (theta)
    double fuelHorizVelocity =
        HUB_SHOT_TREE.calculateShot(robot).flywheelVelocityRotPerSec()
            * HUB_SHOT_TREE.calculateShot(robot).hoodAngle().getCos();
    // let phi be the azimuth
    // phi = arcsin(-V_robot-ground / |V_ball-ground|)
    double phi = Math.asin((-1) * fieldRelativeSpeeds.vyMetersPerSecond / fuelHorizVelocity);
    return Rotation2d.fromRadians(phi);
  }
}
