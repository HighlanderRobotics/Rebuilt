package frc.robot.utils.rusthoundsSOTM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Provides static methods to calculate the effective target position to aim for when shooting on
 * the fly.
 */
public class ShootOnTheFlyCalculator {
  /**
   * Calculates the effective position of the target given the position, velocity, and acceleration
   * of the robot's chassis. Does not account for air resistance, though this is very often
   * unnecessary.
   *
   * <p>When shooting a projectile while moving, the projectile inherits the translational velocity
   * of the chassis. Shooting on the fly can be accomplished by targeting a "virtual" goal if you
   * are moving, which acts to negate the forces applied on the projectile due to the movement of
   * the chassis.
   *
   * <p>An iterative approach (see {@code goalPositionIterations}) is required because the time
   * taken for the projectile to travel to the target will change given a different target location.
   * To account for this, we re-simulate the projectile's travel with a new shot time derived from
   * the new virtual goal position several times.
   *
   * <p>When solving this problem mathematically, the acceleration of the chassis does not matter in
   * the final velocities of the projectile (it will not inherit the acceleration of the chassis).
   * The {@code accelerationCompensationFactor} is necessary due to other errors:
   *
   * <p>(1) the time taken to move the projectile through a shooter is non-zero, so (2) if the
   * chassis is accelerating, the velocity of the chassis by the time the projectile leaves the
   * robot will have changed.
   *
   * <p>To account for this, we multiply the acceleration at the time of the shot <i>command</i> by
   * a specific value and add it to the velocity at the time of the shot. This value is based on the
   * time delta between commanding a shot and the shot actually leaving the shooter, meaning that
   * the effective velocity generated is the velocity as the projectile leaves the shooter. This is
   * extremely complicated to determine theoretically, so if you find acceleration to be causing
   * shot inaccuracies, find a value that provides adequate compensation (should be around [0,2]).
   *
   * <p>The easiest way to create the {@code xyDistanceToProjectileVelocity} lambda function is as
   * follows:
   *
   * <p>If the speed of your shooter is always constant, simply create a lambda expression that
   * always returns the same value.
   *
   * <p>If the speed of your shooter is controlled using an {@link InterpolatingTreeMap} based on
   * distance from the goal, simply get the appropriate shooter speed from that map, and multiply it
   * by some constant that describes how fast the projectile moves given a shooter speed. This can
   * be calculated experimentally by pointing a camera at the shooter and calculating the speed of
   * the projectile based on the distance travelled in n frames. If you find the relationship
   * between shooter speed and projectile speed is not constant, you can create a second {@link
   * InterpolatingTreeMap}, or define it as an equation.
   *
   * @param robotPose the current pose of the robot
   * @param targetPose the 3D pose of the target. this should be the center of the target, if that
   *     makes sense (2024 game), but could also be offset if desired. this may also be deeper into
   *     the target area if required (2020 game). this should take into account any necessary field
   *     reflections before being passed.
   * @param fieldRelRobotVelocity the field-relative velocity of the robot's chassis
   * @param fieldRelRobotAcceleration the field-relative acceleration of the robot's chassis
   * @param xyDistanceToProjectileVelocity a function that takes in the xy-distance from the robot
   *     to the goal, and returns the velocity of the shot projectile in m/s.
   * @param goalPositionIterations the number of iterations to use when iteratively solving for the
   *     pose of the target. a higher number of iterations will increase the accuracy of the result,
   *     but will also reduce performance.
   * @param accelerationCompensationFactor the value to multiply the acceleration
   * @return
   */
  // i 2d ified everything but that scares me a bit
  public static Pose2d calculateEffectiveTargetLocation(
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> fieldRelRobotVelocity,
      // Supplier<ChassisAccelerations> fieldRelRobotAcceleration,
      double goalPositionIterations,
      double accelerationCompensationFactor) {

    double shotTime = AutoAim.HUB_SHOT_TREE.calculateShot(robotPose.get()).timeOfFlightSecs();

    Pose2d correctedTargetPose = new Pose2d();
    for (int i = 0; i < goalPositionIterations; i++) {
      double virtualGoalX =
          FieldUtils.getCurrentHubPose().getY()
              + shotTime
                  * MathUtil.applyDeadband(
                      Math.abs(Math.pow(fieldRelRobotVelocity.get().vxMetersPerSecond, 2))
                          * Math.signum(fieldRelRobotVelocity.get().vxMetersPerSecond),
                      0.1);
      Logger.recordOutput("Autoaim/virtual x", virtualGoalX);
      double virtualGoalY =
          FieldUtils.getCurrentHubPose().getX()
              - shotTime
                  * MathUtil.applyDeadband(
                      Math.abs(Math.pow(fieldRelRobotVelocity.get().vyMetersPerSecond, 2))
                          * Math.signum(fieldRelRobotVelocity.get().vyMetersPerSecond),
                      0.1);
      Logger.recordOutput("Autoaim/virtual y", virtualGoalY);

      correctedTargetPose =
          new Pose2d(virtualGoalY, virtualGoalX, FieldUtils.getCurrentHubPose().getRotation());
      // i don't think rotation even matters here but

      double newShotTime =
          AutoAim.HUB_SHOT_TREE
              .calculateShot(robotPose.get(), correctedTargetPose)
              .timeOfFlightSecs();

      shotTime = newShotTime;
      if (Math.abs(newShotTime - shotTime) <= 0.010) {
        break;
      }
    }
    Logger.recordOutput("Autoaim/Virtual Target", correctedTargetPose);
    Logger.recordOutput(
        "Autoaim/shot",
        new Pose2d(
            robotPose.get().getTranslation().plus(new Translation2d(0, 10)),
            robotPose.get().getRotation()));
    return correctedTargetPose;
  }

  public record InterceptSolution(Pose2d effectiveTargetPose, ShotData shotData) {}

  public static InterceptSolution solveShootOnTheFly(
      Pose2d shooterPose,
      ChassisSpeeds fieldRelRobotVelocity,
      int maxIterations,
      double timeTolerance) {
    ShotData sol = AutoAim.HUB_SHOT_TREE.calculateShot(shooterPose);

    double t = sol.timeOfFlightSecs();
    Pose2d effectiveTarget = FieldUtils.getCurrentHubPose();

    for (int i = 0; i < maxIterations; i++) {

      double dx = fieldRelRobotVelocity.vxMetersPerSecond * t;
      // + 0.5 * fieldRelRobotAcceleration.axMetersPerSecondSquared * t * t;

      double dy = fieldRelRobotVelocity.vyMetersPerSecond * t;
      // + 0.5 * fieldRelRobotAcceleration.ayMetersPerSecondSquared * t * t;

      effectiveTarget =
          new Pose2d(
              FieldUtils.getCurrentHubPose().getX() - dx,
              FieldUtils.getCurrentHubPose().getY() - dy,
              FieldUtils.getCurrentHubPose().getRotation());

      //     ShotSolution newSol = BallPhysics.solveBallisticWithSpeed(
      //             shooterPose,
      //             effectiveTarget,
      //             targetSpeedRps);
      ShotData newSol = AutoAim.HUB_SHOT_TREE.calculateShot(shooterPose, effectiveTarget);

      if (Math.abs(newSol.timeOfFlightSecs() - t) < timeTolerance) {
        return new InterceptSolution(effectiveTarget, newSol);
      }

      sol = newSol;
      t = newSol.timeOfFlightSecs();
    }

    return new InterceptSolution(effectiveTarget, sol);
  }
}
