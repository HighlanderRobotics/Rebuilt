package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import org.littletonrobotics.junction.Logger;

public class AutoAim {

  public static final InterpolatingShotTree HUB_SHOT_TREE = new InterpolatingShotTree();

  // If we need other shot trees (i.e. for feeding) we can put them here

  static { // For hub shot tree
    // 6 feet from edge of hub
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(12 * 2 * Math.sqrt(2) + 6 + 12),
        new ShotData(Rotation2d.fromDegrees(6), 30));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(12 * 2 * Math.sqrt(2) + 6 + 36),
        new ShotData(Rotation2d.fromDegrees(8), 30));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(12 * 2 * Math.sqrt(2) + 6 + 36 + 24),
        new ShotData(Rotation2d.fromDegrees(12), 31));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(12 * 2 * Math.sqrt(2) + 6 + 36 + 24 + 24),
        new ShotData(Rotation2d.fromDegrees(17), 30));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(12 * 2 * Math.sqrt(2) + 6 + 36 + 24 + 24 + 24),
        new ShotData(Rotation2d.fromDegrees(21), 30));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(12 * 2 * Math.sqrt(2) + 6 + 36 + 24 + 24 + 24 + 24),
        new ShotData(Rotation2d.fromDegrees(24), 30));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(12 * 2 * Math.sqrt(2) + 6 + 36 + 24 + 24 + 24 + 24 + 24),
        new ShotData(Rotation2d.fromDegrees(28), 30));

    // 2sqrt2 ft +
    // 6 deg 30 rps
  }

  // Ig we'll see if we need more than 1 feed shot tree
  public static final InterpolatingShotTree FEED_SHOT_TREE = new InterpolatingShotTree();

  static { // For feed shot tree
    // TODO: POPULATE
    FEED_SHOT_TREE.put(
        1.0, new ShotData(Rotation2d.kCW_90deg, 10)); // Placeholder to prevent crashes
  }

  // TODO: SOTM

  public static double distanceToHub(Pose2d pose) {
    double distance = pose.getTranslation().getDistance(FieldUtils.getCurrentHubTranslation());
    Logger.recordOutput("Autoaim/Distance To Hub", distance);
    return distance;
  }
}
