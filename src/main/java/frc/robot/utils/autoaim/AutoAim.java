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
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import org.littletonrobotics.junction.Logger;

public class AutoAim {

  private static boolean outOfRange = false; // TODO not sure if this should be true by default

  public static double LATENCY_COMPENSATION_SECS =
      //   new LoggedTunableNumber("Latency time", 0.3).getAsDouble(); // 0.6; // TODO tune latency
      // comp
      0.0;
  //   public static double SPIN_UP_SECS = 0.0; // TODO tune spinup time

  public static final InterpolatingShotTree ALPHA_HUB_SHOT_TREE = new InterpolatingShotTree();

  public static final Rotation2d LEFT_FIXED_SHOT_TURRET_ANGLE = Rotation2d.fromDegrees(-73.916016);
  public static final Rotation2d MID_FIXED_SHOT_TURRET_ANGLE = Rotation2d.fromDegrees(-82);
  public static final Rotation2d RIGHT_FIXED_SHOT_TURRET_ANGLE =
      Rotation2d.fromDegrees(-109.775391);

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

  static {
    COMP_HUB_SHOT_TREE.put(
        1.716849,
        new ShotData(
            Rotation2d.fromDegrees(23 - 13.16),
            30 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            0.8));
    COMP_HUB_SHOT_TREE.put(
        2.017596,
        new ShotData(
            Rotation2d.fromDegrees(23 - 13.16),
            33 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            0.9));
    COMP_HUB_SHOT_TREE.put(
        2.423868,
        new ShotData(
            Rotation2d.fromDegrees(25 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.1));
    COMP_HUB_SHOT_TREE.put(
        2.664198,
        new ShotData(
            Rotation2d.fromDegrees(26 - 13.16),
            36 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2));
    COMP_HUB_SHOT_TREE.put(
        2.903207,
        new ShotData(
            Rotation2d.fromDegrees(30 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2));
    COMP_HUB_SHOT_TREE.put(
        3.156802,
        new ShotData(
            Rotation2d.fromDegrees(32 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.23));
    COMP_HUB_SHOT_TREE.put(
        3.437033,
        new ShotData(
            Rotation2d.fromDegrees(34 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.25));
    COMP_HUB_SHOT_TREE.put(
        3.611052,
        new ShotData(
            Rotation2d.fromDegrees(38 - 13.16),
            34 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.24));
    COMP_HUB_SHOT_TREE.put(
        3.773999,
        new ShotData(
            Rotation2d.fromDegrees(39 - 13.16),
            34 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.21));
    COMP_HUB_SHOT_TREE.put(
        3.899275,
        new ShotData(
            Rotation2d.fromDegrees(40 - 13.16),
            34 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2));
    COMP_HUB_SHOT_TREE.put(
        4.138058,
        new ShotData(
            Rotation2d.fromDegrees(41 - 13.16),
            34 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.13));
    COMP_HUB_SHOT_TREE.put(
        4.602258,
        new ShotData(
            Rotation2d.fromDegrees(43 - 13.16),
            34.5 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.15));
    COMP_HUB_SHOT_TREE.put(
        4.893493,
        new ShotData(
            Rotation2d.fromDegrees(45 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2));
    COMP_HUB_SHOT_TREE.put(
        5.225402,
        new ShotData(
            Rotation2d.fromDegrees(47 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2));
    COMP_HUB_SHOT_TREE.put(
        5.584793,
        new ShotData(
            Rotation2d.fromDegrees(49 - 13.16),
            35.5 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.17));
  }

  // Ig we'll see if we need more than 1 feed shot tree
  public static final InterpolatingShotTree FEED_SHOT_TREE = new InterpolatingShotTree();

  static { // For feed shot tree
    FEED_SHOT_TREE.put(
        Units.feetToMeters(2), new ShotData(Rotation2d.fromDegrees(23.16), 20 - 2, 0)); // - 2, 0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(4),
        new ShotData(Rotation2d.fromDegrees(30), 40 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(6),
        new ShotData(Rotation2d.fromDegrees(40), 30 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(8),
        new ShotData(Rotation2d.fromDegrees(40), 32 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(10),
        new ShotData(Rotation2d.fromDegrees(40), 35 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(12),
        new ShotData(Rotation2d.fromDegrees(40), 40 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(14),
        new ShotData(Rotation2d.fromDegrees(45), 38 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(16),
        new ShotData(Rotation2d.fromDegrees(45), 40 - 2, 0.0)); // - 2, 0.0));

    FEED_SHOT_TREE.put(
        Units.feetToMeters(18),
        new ShotData(
            Rotation2d.fromDegrees(40 - 13.16),
            36 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.42)); // - 2, 1.42));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(20),
        new ShotData(
            Rotation2d.fromDegrees(43 - 13.16),
            38 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.36)); // - 2, 1.36));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(22),
        new ShotData(
            Rotation2d.fromDegrees(45 - 13.16),
            39 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.34)); // - 2, 1.34));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(24),
        new ShotData(
            Rotation2d.fromDegrees(47 - 13.16),
            40 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.25)); // - 2, 1.25));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(26),
        new ShotData(
            Rotation2d.fromDegrees(48 - 13.16),
            41 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.28)); // - 2, 1.28));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(28),
        new ShotData(
            Rotation2d.fromDegrees(49 - 13.16),
            43 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.27)); // - 2, 1.27));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(30),
        new ShotData(
            Rotation2d.fromDegrees(49 - 13.16),
            44 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.32)); // - 2, 1.32));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(32),
        new ShotData(
            Rotation2d.fromDegrees(49 - 13.16),
            46 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.4)); // - 2, 1.4));

    FEED_SHOT_TREE.put(
        Units.feetToMeters(34),
        new ShotData(
            Rotation2d.fromDegrees(52 - 13.16),
            47 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.3)); // - 2, 1.3));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(36),
        new ShotData(
            Rotation2d.fromDegrees(53 - 13.16),
            51 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.33)); // - 2, 1.33));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(38),
        new ShotData(
            Rotation2d.fromDegrees(53 - 13.16),
            55 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.3)); // - 2, 1.3));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(40),
        new ShotData(
            Rotation2d.fromDegrees(55 - 13.16),
            55 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2)); // - 2, 1.2));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(42),
        new ShotData(
            Rotation2d.fromDegrees(56 - 13.16),
            57 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2)); // - 2, 1.2));

    // TODO: POPULATE beyond 24 feet and time of flight
  }

  public static Pose2d getTurretPose(Pose2d robotPose) {
    Pose2d turretPose =
        robotPose.transformBy(
            new Transform2d(TurretSubsystem.ROBOT_TO_TURRET_TRANSLATION, Rotation2d.kZero));
    return turretPose;
  }

  public static double distanceToHub(Pose2d pose) {
    double distance = pose.getTranslation().getDistance(FieldUtils.getCurrentHubTranslation());
    Logger.recordOutput("Autoaim/Distance To Hub", distance);
    return distance;
  }

  public static Rotation2d getTargetRotation(Translation2d target, Pose2d robotPose) {
    Translation2d robotToTarget = target.minus(robotPose.getTranslation());
    Rotation2d rot = Rotation2d.fromRadians(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
    Logger.recordOutput("Autoaim/Target Rotation", rot);
    return rot;
  }

  public static Rotation2d getVirtualTargetYaw(
      Translation2d target, ChassisSpeeds fieldRelativeSpeeds, Pose2d robotPose, double tof) {
    Translation2d vtarget = getVirtualSOTMTarget(target, fieldRelativeSpeeds, tof);
    return getTargetRotation(vtarget, robotPose);
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
      ChassisSpeeds fieldRelativeSpeeds,
      Translation2d targetTranslation,
      Pose2d robotPose,
      InterpolatingShotTree tree) {
    double tof = tree.calculateShot(robotPose, targetTranslation).timeOfFlightSecs();
    return getVirtualTargetYaw(targetTranslation, fieldRelativeSpeeds, robotPose, tof);
  }

  // if we have a turret im going to assume we're on comp
  public static Rotation2d getTurretTargetRotation(
      Translation2d target,
      Pose2d robotPose,
      ChassisSpeeds chassisSpeeds,
      InterpolatingShotTree shotTree) {
    Pose2d turretPose = getTurretPose(robotPose);

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
    double turretTargetDegrees = turretTargetRotation.getDegrees() - 5;
    // If its in the deadzone, clamp to nearest hardstop
    outOfRange =
        turretTargetDegrees > TurretSubsystem.TURRET_FORWARD_HARDSTOP_ANGLE.getDegrees()
            && (turretTargetDegrees < TurretSubsystem.TURRET_LEFT_HARDSTOP_ANGLE.getDegrees());
    if (outOfRange) {
      turretTargetDegrees =
          // If the requested angle is greater than the halfway point in the deadzone, go to the
          // read hardstop, otherwise go to forward hardstop
          turretTargetDegrees
                  > (TurretSubsystem.TURRET_FORWARD_HARDSTOP_ANGLE.getDegrees()
                          + TurretSubsystem.TURRET_LEFT_HARDSTOP_ANGLE.getDegrees())
                      / 2
              ? TurretSubsystem.TURRET_LEFT_HARDSTOP_ANGLE.getDegrees() + 2
              : TurretSubsystem.TURRET_FORWARD_HARDSTOP_ANGLE.getDegrees() - 2;
    }

    Logger.recordOutput("Turret/Clamped target", Rotation2d.fromDegrees(turretTargetDegrees));
    // Now we need to rewrap this angle to always be negative, with 0 as the forward hardstop
    turretTargetDegrees = MathUtil.inputModulus(turretTargetDegrees, -360, 0);
    Logger.recordOutput("Turret/Wrapped target", Rotation2d.fromDegrees(turretTargetDegrees));
    // ship it
    return Rotation2d.fromDegrees(turretTargetDegrees);
  }

  public static Rotation2d getTurretHubTargetRotation(
      Translation2d target, Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    return getTurretTargetRotation(target, robotPose, chassisSpeeds, COMP_HUB_SHOT_TREE);
  }

  public static Rotation2d getTurretFeedTargetRotation(
      Translation2d target, Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    return getTurretTargetRotation(target, robotPose, chassisSpeeds, FEED_SHOT_TREE);
  }

  public static ShotData getSOTMShotData(
      Pose2d robotPose,
      Translation2d targetTranslation,
      ChassisSpeeds fieldRelativeSpeeds,
      InterpolatingShotTree tree) {

    ShotData unadjustedShot = tree.calculateShot(robotPose, targetTranslation);
    Translation2d virtualTarget =
        getVirtualSOTMTarget(
            targetTranslation, fieldRelativeSpeeds, unadjustedShot.timeOfFlightSecs());
    Pose2d turretPose = getTurretPose(robotPose);

    return tree.get(turretPose.getTranslation().getDistance(virtualTarget));
  }

  public static ShotData getSOTMShotDataNewtonsMethod(
      Pose2d robotPose,
      Translation2d targetTranslation,
      ChassisSpeeds fieldRelativeSpeeds,
      InterpolatingShotTree tree) {

    ShotData baseline = tree.calculateShot(robotPose, targetTranslation);

    Pose2d turretPose = getTurretPose(robotPose);
    Translation2d turretToTarget = targetTranslation.minus(turretPose.getTranslation());

    double distance = turretToTarget.getNorm();

    // get just direction of vector because its vector divded by length
    // dont want to account for magnitude bc the speed we are going and shot tree do
    // and we just want direction to find dot product
    Translation2d shotDirection = turretToTarget.div(distance);

    // dot product! <3
    // get how fast we are going towards where we are shooting
    // vectors of robot times direction
    // positive if going towrds
    // zero is moving perpedicular
    // negative is going away
    double robotVelocityAlongShot =
        fieldRelativeSpeeds.vxMetersPerSecond * shotDirection.getX()
            + fieldRelativeSpeeds.vyMetersPerSecond * shotDirection.getY();

    // required velocity is like velocity the ball must have so it hits the target while the robot
    // moving
    // because the ball velocity is robot velocity + ball velocity
    // so velocity ball needs to go is our distance / tof - the dot product or velocity along that
    // shot to account for the robots velocity along the shot
    double requiredVelocity = (distance / baseline.timeOfFlightSecs()) - robotVelocityAlongShot;

    return calculateShotAdjustments(distance, baseline, requiredVelocity, tree);
  }

  /**
   * @param distance distance to target
   * @param baseline daseline parameters from tree
   * @param requiredVelocity required horizontal velocity magnitude
   * @return adjusted shooter command
   */
  private static ShotData calculateShotAdjustments(
      double distance, ShotData baseline, double requiredVelocity, InterpolatingShotTree tree) {

    ShotData currentParams = baseline;
    double currentDistance = distance;
    double currentTime = currentParams.timeOfFlightSecs();
    double currentVelocity = currentDistance / currentTime;

    // iterate
    for (int i = 0; i < 20; i++) {
      final double EPSILON = 0.001;
      // get deriv of velocity (dis/time)
      double lowVel =
          (currentDistance - EPSILON) / tree.get(currentDistance - EPSILON).timeOfFlightSecs();
      double highVel =
          (currentDistance + EPSILON) / tree.get(currentDistance + EPSILON).timeOfFlightSecs();
      double velDeriv = (highVel - lowVel) / (EPSILON * 2);
      // newtons method: xn+1 = xn - f(xn)/deriv(xn)
      // so estimate for new dist is difference between current velocity required velocity over the
      // deriv
      // this makes sense because if current vel is larger it will lower current distance to account
      // for that and if requird is larger it will increase to account for that
      currentDistance -= (currentVelocity - requiredVelocity) / velDeriv;
      // update
      currentParams = tree.get(currentDistance);
      currentTime = currentParams.timeOfFlightSecs();
      currentVelocity = currentDistance / currentTime;
    }
    return new ShotData(
        currentParams.hoodAngle(),
        currentParams.flywheelVelocityRotPerSec(),
        currentParams.timeOfFlightSecs());
  }

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

  public static boolean targetInTurretDeadzone() {
    return outOfRange;
  }

  public static ShotData getLeftFixedShotData() {
    return new ShotData(Rotation2d.fromDegrees(36), 36, 0);
  }

  public static ShotData getRightFixedShotData() {
    return new ShotData(Rotation2d.fromDegrees(23.16), 35.7, 0);
  }

  public static ShotData getMidFixedShotData() {
    return new ShotData(Rotation2d.fromDegrees(32.84), 35, 0);
  }
}
