package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;

public class AutoAim {

  public static final InterpolatingShotTree HUB_SHOT_TREE = new InterpolatingShotTree();

  // If we need other shot trees (i.e. for feeding) we can put them here

  static { // For hub shot tree
    // 6 feet from edge of hub
    HUB_SHOT_TREE.put(Units.inchesToMeters(95), new ShotData(Rotation2d.fromDegrees(0), 0, 0));
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
}
