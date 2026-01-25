package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import org.littletonrobotics.junction.Logger;

public class AutoAim {

  public static final InterpolatingShotTree HUB_SHOT_TREE = new InterpolatingShotTree();

  static { // For hub shot tree
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 + 17), new ShotData(Rotation2d.fromDegrees(8), 27.5, 1.46, Units.inchesToMeters(24 + 17) / 1.46));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 12),
        new ShotData(Rotation2d.fromDegrees(6), 30, 1.55, Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 12) / 1.55));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 3 * 12),
        new ShotData(Rotation2d.fromDegrees(10.5), 30, 1.54, Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 3 * 12) / 1.54));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 5 * 12),
        new ShotData(Rotation2d.fromDegrees(14.5), 30, 1.54, Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 5 * 12) / 1.54));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 7 * 12),
        new ShotData(Rotation2d.fromDegrees(18), 30, 1.52, Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 7 * 12) / 1.52));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 9 * 12),
        new ShotData(Rotation2d.fromDegrees(21.5), 30, 1.46, Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 9 * 12) / 1.46));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 11 * 12),
        new ShotData(Rotation2d.fromDegrees(24.5), 30, 1.35, Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 11 * 12) / 1.35));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 13 * 12),
        new ShotData(Rotation2d.fromDegrees(28), 30, 1.36, Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 13 * 12) / 1.36));
  }

  // Ig we'll see if we need more than 1 feed shot tree
  public static final InterpolatingShotTree FEED_SHOT_TREE = new InterpolatingShotTree();

  static { // For feed shot tree
    // TODO: POPULATE
    FEED_SHOT_TREE.put(
        1.0, new ShotData(Rotation2d.kCW_90deg, 10, 0, 0)); // Placeholder to prevent crashes
  }

  // TODO: SOTM

  public static double distanceToHub(Pose2d pose) {
    double distance = pose.getTranslation().getDistance(FieldUtils.getCurrentHubTranslation());
    Logger.recordOutput("Autoaim/Distance To Hub", distance);
    return distance;
  }

  public static Rotation2d getSOTMYaw(Pose2d robot, ChassisSpeeds fieldRelativeSpeeds) {
    // V_ball-ground = V_ball-robot + V_robot-ground (relative motion)
    // if we want the ball to go straight towards the goal,
    // the V_ball-robot vector needs to cancel out with the V_robot-ground vector to "offset" the
    // velocity it already has
    //this is the desired final ground velocity of the ball
    double v_ballGround =
        HUB_SHOT_TREE.calculateShot(robot).groundVelocity();
    // let phi be the azimuth
    // phi = arcsin(-V_robot-ground / |V_ball-ground|)
    double phi = Math.asin((-1) * fieldRelativeSpeeds.vyMetersPerSecond / v_ballGround);
    return Rotation2d.fromRadians(phi);
  }
}
