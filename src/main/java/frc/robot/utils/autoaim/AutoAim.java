package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.FieldUtils;

public class AutoAim {

  public static final InterpolatingShotTree HUB_SHOT_TREE = new InterpolatingShotTree();

  // If we need other shot trees (i.e. for feeding) we can put them here

  static {
    // TODO: ADD SHOTS TO HUB SHOT HERE
  }

  // TODO: SOTM

  public static double distanceToHub(Pose2d pose) {
    return pose.getTranslation().getDistance(FieldUtils.getCurrentHubTranslation());
  }
}
