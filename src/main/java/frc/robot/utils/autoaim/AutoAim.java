package frc.robot.utils.autoaim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import org.littletonrobotics.junction.Logger;

public class AutoAim {

  public static double LATENCY_COMPENSATION_SECS =
      new LoggedTunableNumber("Latency time", 0.0).getAsDouble(); // 0.6; // TODO tune latency comp
  //   public static double SPIN_UP_SECS = 0.0; // TODO tune spinup time

  public static final InterpolatingShotTree ALPHA_HUB_SHOT_TREE = new InterpolatingShotTree();

  static { // For hub shot tree
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 + 17), new ShotData(Rotation2d.fromDegrees(8), 27.5, 1.46));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 12),
        new ShotData(Rotation2d.fromDegrees(6), 30, 1.55));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 3 * 12),
        new ShotData(Rotation2d.fromDegrees(10.5), 30, 1.54));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 5 * 12),
        new ShotData(Rotation2d.fromDegrees(14.5), 30, 1.54));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 7 * 12),
        new ShotData(Rotation2d.fromDegrees(18.25), 30, 1.52));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 9 * 12),
        new ShotData(Rotation2d.fromDegrees(21.5), 30, 1.46));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 11 * 12),
        new ShotData(Rotation2d.fromDegrees(24.5), 30, 1.35));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 13 * 12),
        new ShotData(Rotation2d.fromDegrees(28), 30, 1.36));
  }

  public static final InterpolatingShotTree COMP_HUB_SHOT_TREE = new InterpolatingShotTree();

  // TODO update tof
  static { // For hub shot tree
    COMP_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 + 17), new ShotData(TurretSubsystem.HOOD_MIN_ANGLE, 40, 1.04));

    COMP_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 12),
        new ShotData(Rotation2d.fromDegrees(32), 40, 1.14));

    COMP_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 3 * 12),
        new ShotData(Rotation2d.fromDegrees(34), 40, 1.10));

    COMP_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 5 * 12),
        new ShotData(Rotation2d.fromDegrees(38), 40, 1.09));

    COMP_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 7 * 12),
        new ShotData(Rotation2d.fromDegrees(41), 40, 1.15));

    COMP_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 9 * 12),
        new ShotData(Rotation2d.fromDegrees(43), 42, 1.23));

    COMP_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 11 * 12),
        new ShotData(Rotation2d.fromDegrees(45), 44, 1.30));
    COMP_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 13 * 12),
        new ShotData(Rotation2d.fromDegrees(48), 46, 1.35));
  }

  // Ig we'll see if we need more than 1 feed shot tree
  public static final InterpolatingShotTree FEED_SHOT_TREE = new InterpolatingShotTree();

  static { // For feed shot tree
    FEED_SHOT_TREE.put(Units.feetToMeters(2), new ShotData(Rotation2d.fromDegrees(23.16), 20, 0));
    FEED_SHOT_TREE.put(Units.feetToMeters(4), new ShotData(Rotation2d.fromDegrees(30), 40, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(6), new ShotData(Rotation2d.fromDegrees(40), 30, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(8), new ShotData(Rotation2d.fromDegrees(40), 32, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(10), new ShotData(Rotation2d.fromDegrees(40), 35, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(12), new ShotData(Rotation2d.fromDegrees(40), 40, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(14), new ShotData(Rotation2d.fromDegrees(45), 38, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(16), new ShotData(Rotation2d.fromDegrees(45), 40, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(18), new ShotData(Rotation2d.fromDegrees(50), 40, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(20), new ShotData(Rotation2d.fromDegrees(55), 40, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(22), new ShotData(Rotation2d.fromDegrees(55), 44, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(24), new ShotData(Rotation2d.fromDegrees(60), 44, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(26), new ShotData(Rotation2d.fromDegrees(60), 47, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(28), new ShotData(Rotation2d.fromDegrees(60), 58, 0.0));
    FEED_SHOT_TREE.put(Units.feetToMeters(30), new ShotData(Rotation2d.fromDegrees(60), 50, 0.0));
    // TODO: POPULATE beyond 24 feet and time of flight
  }

  /**
   * Gets the distance from the passed-in pose to the hub
   * @param pose
   * @return the distance from the passed-in pose to the hub
   */
  public static double distanceToHub(Pose2d pose) {
    double distance = pose.getTranslation().getDistance(FieldUtils.getCurrentHubTranslation());
    Logger.recordOutput("Autoaim/Distance To Hub", distance);
    return distance;
  }

  // lock in
  // TODO: BETTER DOC COMMENT
  /**
   * Returns the position of the target translated by the distance moved by the robot in the passed in time of flight
   * @param target the target position to translate
   * @param fieldRelativeSpeeds the field relative robot speeds
   * @param timeOfFlightSecs the time the ball will spend in the air after it's shot
   * @return the translated target
   */
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

  /**
   * Gets the (world relative) yaw required to shoot at the passed in target while moving at the passed in speed.
   * @param target the (world relative) target location
   * @param fieldRelativeSpeeds the speeds the robot is moving at
   * @param robotPose the pose of the robot
   * @param tof the time of flight of the shot
   * @return the target rotation required to hit the passed in target
   */
  public static Rotation2d getVirtualTargetYaw(
      Translation2d target, ChassisSpeeds fieldRelativeSpeeds, Pose2d robotPose, double tof) {
    Translation2d vtarget = getVirtualSOTMTarget(target, fieldRelativeSpeeds, tof);
    return getTargetRotation(vtarget, robotPose);
  }

  /**
   * Gets the (world relative) yaw required to point at the passed-in target
   * @param target the target to point at
   * @param robotPose the current location of the robot
   * @return
   */
  public static Rotation2d getTargetRotation(Translation2d target, Pose2d robotPose) {
    Translation2d robotToTarget = target.minus(robotPose.getTranslation());
    Rotation2d rot = Rotation2d.fromRadians(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
    Logger.recordOutput("Autoaim/Target Rotation", rot);
    return rot;
  }

  // if we have a turret im going to assume we're on comp
  /**
   * Gets the required turret motor position (i.e. robot relative) to shoot at the passed-in target
   * @param target the target location (i.e. the hub or a feed location)
   * @param robotPose the current pose of the robot (translation and rotatio)
   * @param chassisSpeeds the current field relative speeds of the robot
   * @param shotTree the shot tree to calculate the shots with
   * @return the rotation of the turret required to hit the passed-in target
   */
  public static Rotation2d getTurretTargetRotation(
      Translation2d target,
      Pose2d robotPose,
      ChassisSpeeds chassisSpeeds,
      InterpolatingShotTree shotTree) {
    Pose2d turretPose =
        robotPose.transformBy(
            new Transform2d(TurretSubsystem.ROBOT_TO_TURRET_TRANSLATION, Rotation2d.kZero));

    // get desired rotation to point at target
    Rotation2d turretTargetRotation =
        AutoAim.getVirtualTargetYaw(chassisSpeeds, target, turretPose, shotTree);
    // subtract that from rotation to point at target
    turretTargetRotation = turretTargetRotation.minus(robotPose.getRotation());
    Logger.recordOutput("Turret/Unclamped target", turretTargetRotation);
    // clamp between min and max turret angle
    // turretTargetRotations =
    //     MathUtil.clamp(
    //         turretTargetRotations,
    //         TurretSubsystem.TURRET_MIN_ANGLE.getRotations(),
    //         TurretSubsystem.TURRET_MAX_ANGLE.getRotations());
    double turretTargetDegrees = turretTargetRotation.getDegrees();
    // If its in the deadzone, clamp to nearest hardstop
    if (turretTargetDegrees > TurretSubsystem.TURRET_FORWARD_HARDSTOP_ANGLE.getDegrees()
        && (turretTargetDegrees < TurretSubsystem.TURRET_REAR_HARDSTOP_ANGLE.getDegrees())) {
      turretTargetDegrees =
          // If the requested angle is greater than the halfway point in the deadzone, go to the
          // read hardstop, otherwise go to forward hardstop
          turretTargetDegrees
                  > (TurretSubsystem.TURRET_FORWARD_HARDSTOP_ANGLE.getDegrees()
                          + TurretSubsystem.TURRET_REAR_HARDSTOP_ANGLE.getDegrees())
                      / 2
              ? TurretSubsystem.TURRET_REAR_HARDSTOP_ANGLE.getDegrees()
              : TurretSubsystem.TURRET_FORWARD_HARDSTOP_ANGLE.getDegrees();
    }

    Logger.recordOutput("Turret/Clamped target", Rotation2d.fromDegrees(turretTargetDegrees));
    // Now we need to rewrap this angle to always be negative, with 0 as the forward hardstop
    turretTargetDegrees = MathUtil.inputModulus(turretTargetDegrees, -360, 0);
    Logger.recordOutput("Turret/Wrapped target", Rotation2d.fromDegrees(turretTargetDegrees));
    // ship it
    return Rotation2d.fromDegrees(turretTargetDegrees);
  }

  /**
   * Gets the turret position required to hit the hub, using the the hub shot tree
   * @param target the target hub
   * @param robotPose the robot position
   * @param chassisSpeeds current velocity of the robot
   * @return the turret position to hit the target
   */
  public static Rotation2d getTurretHubTargetRotation(
      Translation2d target, Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    return getTurretTargetRotation(target, robotPose, chassisSpeeds, COMP_HUB_SHOT_TREE);
  }

  /**
   * Gets the turret position required to hit the passed in feed target, using the feed shot tree
   * @param target the target feed location
   * @param robotPose the current robot position
   * @param chassisSpeeds the current robot velocity
   * @return the turret position to hit the target
   */
  public static Rotation2d getTurretFeedTargetRotation(
      Translation2d target, Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    return getTurretTargetRotation(target, robotPose, chassisSpeeds, FEED_SHOT_TREE);
  }

  /**
   * Gets the required yaw to hit the target, given a shot tree (instead of TOF)
   * @see #getVirtualTargetYaw(Translation2d, ChassisSpeeds, Pose2d, double)
   * @param fieldRelativeSpeeds
   * @param targetTranslation
   * @param robotPose
   * @param tree
   * @return the required rotation to hit the target
   */
  public static Rotation2d getVirtualTargetYaw(
      ChassisSpeeds fieldRelativeSpeeds,
      Translation2d targetTranslation,
      Pose2d robotPose,
      InterpolatingShotTree tree) {
    double tof = tree.calculateShot(robotPose, targetTranslation).timeOfFlightSecs();
    return getVirtualTargetYaw(targetTranslation, fieldRelativeSpeeds, robotPose, tof);
  }

  /**
   * Returns the {@link ShotData} to hit the passed-in target
   * @param robotPose the current robot position
   * @param targetTranslation the location (world relative) of the target
   * @param fieldRelativeSpeeds the velocity of the robot
   * @param tree the shot tree to calculate the shot data off of
   * @return 
   */
  public static ShotData getSOTMShotData(
      Pose2d robotPose,
      Translation2d targetTranslation,
      ChassisSpeeds fieldRelativeSpeeds,
      InterpolatingShotTree tree) {
    ShotData unadjustedShot = tree.calculateShot(robotPose, targetTranslation);
    Translation2d virtualTarget =
        getVirtualSOTMTarget(
            targetTranslation, fieldRelativeSpeeds, unadjustedShot.timeOfFlightSecs());
    return tree.get(robotPose.getTranslation().getDistance(virtualTarget));
  }

  /**
   * Returns the {@link ShotData} to hit the passed-in target compensating for latency
   * @param robotPose the current robot position
   * @param targetTranslation the location (world relative) of the target
   * @param fieldRelativeSpeeds the velocity of the robot
   * @param tree the shot tree to calculate the shot data off of
   * @return
   */
  public static ShotData getCompensatedSOTMShotData(
      Pose2d robotPose,
      Translation2d targetTranslation,
      ChassisSpeeds fieldRelativeSpeeds,
      InterpolatingShotTree tree) {
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
    return getSOTMShotData(compensatedPose, targetTranslation, fieldRelativeSpeeds, tree);
  }
}
