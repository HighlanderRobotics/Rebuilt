package frc.robot.utils.autoaim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.TreeMap;

public class InterpolatingShotTree {
  private record ShotData(
      Rotation2d hoodRotation, double flywheelVelocityRotPerSec, double flightTimeSec) {}

  private final TreeMap<Double, ShotData> map = new TreeMap<>();

  public InterpolatingShotTree() {}

  public void put(Double distance, ShotData data) {
    map.put(distance, data);
  }

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

  public void clear() {
    map.clear();
  }

  public void remove(double key) {
    map.remove(key);
  }

  public double maxKey() {
    return map.lastKey();
  }

  private ShotData interpolate(ShotData startValue, ShotData endValue, double t) {
    return new ShotData(
        Rotation2d.fromRadians(
            MathUtil.interpolate(
                startValue.hoodRotation().getRadians(), endValue.hoodRotation().getRadians(), t)),
        MathUtil.interpolate(
            startValue.flywheelVelocityRotPerSec(), endValue.flywheelVelocityRotPerSec(), t),
        MathUtil.interpolate(startValue.flightTimeSec(), endValue.flightTimeSec(), t));
  }

  private double inverseInterpolate(Double up, Double q, Double down) {
    double upperToLower = up.doubleValue() - down.doubleValue();
    if (upperToLower <= 0) {
      return 0.0;
    }
    double queryToLower = q.doubleValue() - down.doubleValue();
    if (queryToLower <= 0) {
      return 0.0;
    }
    return queryToLower / upperToLower;
  }
}
