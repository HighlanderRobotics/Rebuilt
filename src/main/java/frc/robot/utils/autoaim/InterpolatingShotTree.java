package frc.robot.utils.autoaim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.TreeMap;

public class InterpolatingShotTree {
  public record ShotData(
      Rotation2d hoodRotation, double flywheelVelocityRotPerSec, double flightTimeSec) {}

  private final TreeMap<Double, ShotData> map = new TreeMap<>();

  public InterpolatingShotTree() {}

  /**
   * Add an entry to the shot map
   * @param distance the distance from the goal of the shot
   * @param data the data that made the shot
   */
  public void put(Double distance, ShotData data) {
    map.put(distance, data);
  }

  /**
   * Get shot data for a working shot at the specified distance. If the distance is included in the map, it will return that data. If not, it interpolates a working shot from the nearest data.
   * @param distance the distance from the goal
   * @return 
   */
  public ShotData get(Double distance) {
    ShotData val = map.get(distance);
    if (val == null) {
      Double ceilingKey = map.ceilingKey(distance);
      Double floorKey = map.floorKey(distance);

      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return map.get(floorKey);
      }
      if (floorKey == null) {
        return map.get(ceilingKey);
      }
      ShotData floor = map.get(floorKey);
      ShotData ceiling = map.get(ceilingKey);

      return interpolate(floor, ceiling, inverseInterpolate(ceilingKey, distance, floorKey));
    } else {
      return val;
    }
  }

  /**
   * Clears the shot map
   */
  public void clear() {
    map.clear();
  }

  /**
   * Remove an entry from the shot map
   * @param key the distance from the goal of the entry to remove
   */
  public void remove(double key) {
    map.remove(key);
  }

  /**
   * Gets the largest key in the map
   * @return
   */
  public double maxKey() {
    return map.lastKey();
  }

  /**
   * Interpolates a shot from the two specified shots
   * @param startValue
   * @param endValue
   * @param t how far between the two values to interpolate
   * @return
   */
  private ShotData interpolate(ShotData startValue, ShotData endValue, double t) {
    return new ShotData(
        Rotation2d.fromRadians(
            MathUtil.interpolate(
                startValue.hoodRotation().getRadians(), endValue.hoodRotation().getRadians(), t)),
        MathUtil.interpolate(
            startValue.flywheelVelocityRotPerSec(), endValue.flywheelVelocityRotPerSec(), t),
        MathUtil.interpolate(startValue.flightTimeSec(), endValue.flightTimeSec(), t));
  }

  /**
   * Returns the relative distance of the query from the lower key and the larger key from the lower key.
   * @param up the larger key
   * @param query the query key
   * @param down the smaller key
   * @return
   */
  private double inverseInterpolate(Double up, Double query, Double down) {
    double upperToLower = up.doubleValue() - down.doubleValue();
    if (upperToLower <= 0) {
      return 0.0;
    }
    double queryToLower = query.doubleValue() - down.doubleValue();
    if (queryToLower <= 0) {
      return 0.0;
    }
    return queryToLower / upperToLower;
  }

  public ShotData calculateShot(InterpolatingShotTree tree, Pose3d goal, Pose3d shooterPose) {        
        return tree.get(Math.hypot(goal.getX() - shooterPose.getX(), goal.getY() - shooterPose.getY()));
    }
}
