package frc.robot.utils.autoaim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import org.littletonrobotics.junction.Logger;

public class AutoAim {

  public record ShotParams(ShotData shotData, Rotation2d turretAngle) {}

  private static boolean outOfRange = false;

  private static int fudgeFactor = 0;

  //public static double forceFeedForward = 0.0;

    public static LoggedTunableNumber forceFeedForward = new LoggedTunableNumber("force feed forward value", 1.0);

  private static double lastVxMetersPerSec = 0.0;
  private static double lastVyMetersPerSec = 0.0;
  private static double lastOmegaRadPerSec = 0.0;
  private static double lastRunTimeSec = 0.0;

  public static double LATENCY_COMPENSATION_SECS =
      //   new LoggedTunableNumber("Latency time", 0.3).getAsDouble();
      // comp
      0.0;

  

  // TODO: FIX ROTATION AND REDUCE DEFENDABLILTY
  /**
   * Gets the required ShotParams (ShotData and turret rotation) required to fire a ball into the
   * hub, calculating a shot based on the passed in position, velocity, and shot tree.
   *
   * @param estimatedPose the robot pose estimated by the odometry and vision
   * @param robotRelativeVelocity the robot relative ChassisSpeeds the robot is driving at
   * @param target the target to shoot towards
   * @param tree the InterpolatingShotTree used for this target (i.e. for feeding or scoring),
   *     populated with working measurements from known distances
   * @return the ShotParams to hit the target
   */
  public static ShotParams getShotParameters(
      Pose2d estimatedPose,
      ChassisSpeeds robotRelativeVelocity,
      Translation2d target,
      InterpolatingShotTree tree) {

    double currentTimeSec = Timer.getFPGATimestamp();
    double deltaTime = currentTimeSec - lastRunTimeSec;

    double axMetersPerSecSq =
        (robotRelativeVelocity.vxMetersPerSecond - lastVxMetersPerSec) / deltaTime;
    double ayMetersPerSecSq =
        (robotRelativeVelocity.vyMetersPerSecond - lastVyMetersPerSec) / deltaTime;
    double alphaRadPerSecSq =
        (robotRelativeVelocity.omegaRadiansPerSecond - lastOmegaRadPerSec) / deltaTime;

    lastVxMetersPerSec = robotRelativeVelocity.vxMetersPerSecond;
    lastVyMetersPerSec = robotRelativeVelocity.vyMetersPerSecond;
    lastOmegaRadPerSec = robotRelativeVelocity.omegaRadiansPerSecond;

    lastRunTimeSec = currentTimeSec;

    // Calculate estimated pose while accounting movement and acceleration during phase delay
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                (robotRelativeVelocity.vxMetersPerSecond * LATENCY_COMPENSATION_SECS)
                    + (0.5 * axMetersPerSecSq * Math.pow(LATENCY_COMPENSATION_SECS, 2)),
                (robotRelativeVelocity.vyMetersPerSecond * LATENCY_COMPENSATION_SECS)
                    + (0.5 * ayMetersPerSecSq * Math.pow(LATENCY_COMPENSATION_SECS, 2)),
                (robotRelativeVelocity.omegaRadiansPerSecond * LATENCY_COMPENSATION_SECS)
                    + (0.5 * alphaRadPerSecSq * Math.pow(LATENCY_COMPENSATION_SECS, 2))));

    // Calculate turret position
    Pose2d turretPosition =
        estimatedPose.transformBy(
            new Transform2d(TurretSubsystem.ROBOT_TO_TURRET_TRANSLATION, Rotation2d.kZero));
    // Calculate distance from turret to target
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate angle of linear velocity from angular velocity
    double turretRadiusMeters =
        Math.hypot(
            TurretSubsystem.ROBOT_TO_TURRET_TRANSLATION.getX(),
            TurretSubsystem.ROBOT_TO_TURRET_TRANSLATION.getY());
    Rotation2d turretToRobotAngleRads =
        Rotation2d.fromRadians(
            Math.atan2(
                TurretSubsystem.ROBOT_TO_TURRET_TRANSLATION.getY(),
                TurretSubsystem.ROBOT_TO_TURRET_TRANSLATION.getX()));
    Rotation2d turretLinearVelAngle = turretToRobotAngleRads.minus(Rotation2d.kCCW_90deg);

    // Calculate turret velocity, accounting for angular velocity
    double turretVelocityX =
        robotRelativeVelocity.vxMetersPerSecond
            + (robotRelativeVelocity.omegaRadiansPerSecond
                * turretRadiusMeters
                * turretLinearVelAngle.getSin());
    double turretVelocityY =
        robotRelativeVelocity.vyMetersPerSecond
            + (robotRelativeVelocity.omegaRadiansPerSecond
                * turretRadiusMeters
                * turretLinearVelAngle.getCos());
    Logger.recordOutput(
        "LaunchCalculator/Turret Velocity", new Translation2d(turretVelocityX, turretVelocityY));
    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      // Find time of flight for a shot from the current lookahead pose
      timeOfFlight = tree.get(lookaheadTurretToTargetDistance).timeOfFlightSecs();
      // Extrapolate velocity over time of flight of the shot
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;

      Logger.recordOutput("LaunchCalculator/Offset", new Translation2d(offsetX, offsetY));
      // Update lookahead pose
      lookaheadPose =
          turretPosition.transformBy(new Transform2d(offsetX, offsetY, Rotation2d.kZero));
      // Update distance
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    // Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();\
    Rotation2d turretAngle = getTargetRotation(target, lookaheadPose);
    turretAngle = getTurretTargetRotation(turretAngle, estimatedPose);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return new ShotParams(tree.get(lookaheadTurretToTargetDistance), turretAngle);
  }

  /**
   * Calculates the required turret position to point the turret along the passed in yaw. Clamps the
   * rotation to within the deadzone
   *
   * @param targetRotation the target yaw
   * @param robotPose the robot's position and rotation
   * @return the position the turret should go to to down the passed in rotation
   */
  public static Rotation2d getTurretTargetRotation(Rotation2d targetRotation, Pose2d robotPose) {

    // subtract that from rotation to point at target
    Rotation2d turretTargetRotation = targetRotation.minus(robotPose.getRotation());
    Logger.recordOutput("Turret/Unclamped target", turretTargetRotation);
    // -5 is some insane fudge factor i forgot where it's from
    double turretTargetDegrees =
        turretTargetRotation.getDegrees() - 5 - 2.5; // fudge factor of doom and despair
    // If its in the deadzone, clamp to nearest hardstop
    outOfRange =
        turretTargetDegrees > TurretSubsystem.TURRET_FORWARD_HARDSTOP_ANGLE.getDegrees()
            && (turretTargetDegrees < TurretSubsystem.TURRET_LEFT_HARDSTOP_ANGLE.getDegrees());
    if (outOfRange) {
      turretTargetDegrees =
          // If the requested angle is greater than the halfway point in the deadzone, go to the
          // left hardstop, otherwise go to forward hardstop
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

  /**
   * Gets the rotation required to point at the target, given the current position
   *
   * @param target the target to point at
   * @param robotPose the current position
   * @return the rotation required to point at the target
   */
  public static Rotation2d getTargetRotation(Translation2d target, Pose2d robotPose) {
    Translation2d robotToTarget = target.minus(robotPose.getTranslation());
    Rotation2d rot = Rotation2d.fromRadians(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
    Logger.recordOutput("AutoAlign/Target Rotation", rot);
    return rot;
  }

  /**
   * Returns the absolute distance between the passed in pose and the current alliance hub
   *
   * @param pose
   * @return the pose's distance to the hub
   */
  public static double distanceToHub(Pose2d pose) {
    return pose.getTranslation().getDistance(FieldUtils.getCurrentHubTranslation());
  }

  /** Increase the flywheel fudge factor by 1 */
  public static void incrementFudgeFactor() {
    fudgeFactor++;
  }

  /** Decrease the flywheel fudge factor by 1 */
  public static void decrementFudgeFactor() {
    fudgeFactor--;
  }

  /** Get the current flywheel fudge factor */
  public static int getFudgeFactor() {
    return fudgeFactor;
  }

  /** Get the force flywheel feed forward */
  // public static double getForceFeedForward() {
  //   return forceFeedForward;
  // }

  // public static void ForceFeedForward(double fff) {
  //   forceFeedForward = fff;
  // }

  /**
   * Returns whether or not the current target is in the turret deadzone
   *
   * @return true if the target is in the deadzone, false if not
   */
  public static boolean targetInTurretDeadzone() {
    return outOfRange;
  }
}
