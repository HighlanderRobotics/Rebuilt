package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import org.littletonrobotics.junction.Logger;

public class AutoAim {

  public static double LATENCY_COMPENSATION_SECS =
      new LoggedTunableNumber("Latency time", 0.0).getAsDouble(); // 0.6; // TODO tune latency comp
  //   public static double SPIN_UP_SECS = 0.0; // TODO tune spinup time

  public static final InterpolatingShotTree HUB_SHOT_TREE = new InterpolatingShotTree();

  static { // For hub shot tree
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 + 17), new ShotData(Rotation2d.fromDegrees(8), 27.5, 1.46));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 12),
        new ShotData(Rotation2d.fromDegrees(6), 30, 1.55));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 3 * 12),
        new ShotData(Rotation2d.fromDegrees(10.5), 30, 1.54));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 5 * 12),
        new ShotData(Rotation2d.fromDegrees(14.5), 30, 1.54));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 7 * 12),
        new ShotData(Rotation2d.fromDegrees(18.25), 30, 1.52));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 9 * 12),
        new ShotData(Rotation2d.fromDegrees(21.5), 30, 1.46));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 11 * 12),
        new ShotData(Rotation2d.fromDegrees(24.5), 30, 1.35));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 13 * 12),
        new ShotData(Rotation2d.fromDegrees(28), 30, 1.36));
  }

  // Ig we'll see if we need more than 1 feed shot tree
  public static final InterpolatingShotTree FEED_SHOT_TREE = new InterpolatingShotTree();

  static { // For feed shot tree
    // TODO: POPULATE
    FEED_SHOT_TREE.put(
        1.0, new ShotData(Rotation2d.kCW_90deg, 10, 0)); // Placeholder to prevent crashes
  }

  public static double distanceToHub(Pose2d pose) {
    double distance = pose.getTranslation().getDistance(FieldUtils.getCurrentHubTranslation());
    Logger.recordOutput("Autoaim/Distance To Hub", distance);
    return distance;
  }

  // lock in
  public static Translation2d getVirtualSOTMTarget(
      Translation2d target, ChassisSpeeds fieldRelativeSpeeds, double timeOfFlightSecs) {
    // velocity times shot time is how translated it is
    Translation2d vtarget =
        target.minus(
            new Translation2d(
                fieldRelativeSpeeds.vxMetersPerSecond * timeOfFlightSecs,
                fieldRelativeSpeeds.vyMetersPerSecond * timeOfFlightSecs));
    Logger.recordOutput("Autoaim/Virtual Target", vtarget);
    return vtarget;
  }

  public static Rotation2d getVirtualTargetYaw(
      Translation2d target, ChassisSpeeds fieldRelativeSpeeds, Pose2d robotPose) {
    double tof = HUB_SHOT_TREE.calculateShot(robotPose, target).timeOfFlightSecs();
    Translation2d vtarget = getVirtualSOTMTarget(target, fieldRelativeSpeeds, tof);
    return getTargetRotation(vtarget, robotPose);
  }

  public static Rotation2d getTargetRotation(Translation2d target, Pose2d robotPose) {
    Translation2d robotToTarget = target.minus(robotPose.getTranslation());
    Rotation2d rot = Rotation2d.fromRadians(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
    // .plus(Rotation2d.k180deg);
    Logger.recordOutput("Autoaim/Target Rotation", rot);
    return rot;
  }

  // this should also adjust for like the turret offset from the robot rotation but like I don't
  // know what that is
  public static Rotation2d getTurretTargetRotation(Translation2d target, Pose2d robotPose) {
    Rotation2d rot = getTargetRotation(target, robotPose).minus(robotPose.getRotation());
    return rot;
  }

  public static Rotation2d getVirtualHubYaw(ChassisSpeeds fieldRelativeSpeeds, Pose2d robotPose) {
    return getVirtualTargetYaw(
        FieldUtils.getCurrentHubTranslation(), fieldRelativeSpeeds, robotPose);
  }

  public static ShotData getSOTMShotData(
      Pose2d robotPose, Translation2d targetTranslation, ChassisSpeeds fieldRelativeSpeeds) {
    ShotData unadjustedShot = HUB_SHOT_TREE.calculateShot(robotPose, targetTranslation);
    Translation2d virtualTarget =
        getVirtualSOTMTarget(
            targetTranslation, fieldRelativeSpeeds, unadjustedShot.timeOfFlightSecs());
    return HUB_SHOT_TREE.get(robotPose.getTranslation().getDistance(virtualTarget));
  }

  public static ShotData getCompensatedSOTMShotData(
      Pose2d robotPose, Translation2d targetTranslation, ChassisSpeeds fieldRelativeSpeeds) {
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, robotPose.getRotation());
    // calculate latency compensated pose
    Pose2d compensatedPose =
        robotPose.exp(
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond
                    * (LATENCY_COMPENSATION_SECS
                    //  + SPIN_UP_SECS
                    ),
                robotRelativeSpeeds.vyMetersPerSecond
                    * (LATENCY_COMPENSATION_SECS
                    //  + SPIN_UP_SECS
                    ),
                robotRelativeSpeeds.omegaRadiansPerSecond
                    * (LATENCY_COMPENSATION_SECS
                    // + SPIN_UP_SECS
                    )));
    return getSOTMShotData(compensatedPose, targetTranslation, fieldRelativeSpeeds);
  }
}
