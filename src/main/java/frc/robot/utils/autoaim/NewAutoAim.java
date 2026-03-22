// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class NewAutoAim {
  private static boolean outOfRange = false; // TODO not sure if this should be true by default

  private static double lastVxMetersPerSec = 0.0;
  private static double lastVyMetersPerSec = 0.0;
  private static double lastOmegaRadPerSec = 0.0;
  private static double lastRunTimeSec = 0.0;

  public record ShotParams(ShotData shotData, Rotation2d turretAngle) {}

  public static ShotParams calculate(
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      Translation2d target,
      InterpolatingShotTree tree) {
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, robotPose.getRotation());
    // this is robot relative speeds because "The twist is a change in pose in the robot's
    // coordinate frame"
    Pose2d compensatedPose =
        robotPose.exp(
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * (AutoAim.LATENCY_COMPENSATION_SECS),
                robotRelativeSpeeds.vyMetersPerSecond * (AutoAim.LATENCY_COMPENSATION_SECS),
                robotRelativeSpeeds.omegaRadiansPerSecond * (AutoAim.LATENCY_COMPENSATION_SECS)));
    Pose2d turretPose = AutoAim.getTurretPose(compensatedPose);
    double distanceToTarget = robotPose.getTranslation().getDistance(target);

    // the turret isn't in the center of the robot, so if the robot is rotating the turret's x and y
    // are changing slightly
    double turretFieldRelativeVelocityX =
        fieldRelativeSpeeds.vxMetersPerSecond
            + fieldRelativeSpeeds.omegaRadiansPerSecond
                * (TurretSubsystem.ROBOT_TO_TURRET_TRANSLATION.getY()
                        * Math.cos(compensatedPose.getRotation().getRadians())
                    - TurretSubsystem.ROBOT_TO_TURRET_TRANSLATION.getX()
                        * Math.sin(compensatedPose.getRotation().getRadians()));
    double turretFieldRelativeVelocityY =
        fieldRelativeSpeeds.vyMetersPerSecond
            + fieldRelativeSpeeds.omegaRadiansPerSecond
                * (TurretSubsystem.ROBOT_TO_TURRET_TRANSLATION.getX()
                        * Math.cos(compensatedPose.getRotation().getRadians())
                    - TurretSubsystem.ROBOT_TO_TURRET_TRANSLATION.getY()
                        * Math.sin(compensatedPose.getRotation().getRadians()));

    Pose2d lookaheadPose = turretPose;

    // for (int i = 0; i < 20; i++) {
    //   double tof = tree.get(distanceToTarget).timeOfFlightSecs();
    //   lookaheadPose =
    //       turretPose.transformBy(
    //           new Transform2d(
    //               new Translation2d(turretFieldRelativeVelocityX * tof,
    // turretFieldRelativeVelocityY * tof),
    //               Rotation2d.kZero));
    //   distanceToTarget = lookaheadPose.getTranslation().getDistance(target);
    // }
    double tof = tree.get(distanceToTarget).timeOfFlightSecs();

    ShotData baseline = tree.calculateShot(robotPose, target);

    Translation2d turretToTarget = target.minus(turretPose.getTranslation());

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
    double turretVelocityAlongShot =
        turretFieldRelativeVelocityX * shotDirection.getX()
            + turretFieldRelativeVelocityY * shotDirection.getY();

    // required velocity is like velocity the ball must have so it hits the target while the robot
    // moving
    // because the ball velocity is robot velocity + ball velocity
    // so velocity ball needs to go is our distance / tof - the dot product or velocity along that
    // shot to account for the robots velocity along the shot
    double requiredVelocity = (distance / baseline.timeOfFlightSecs()) - turretVelocityAlongShot;

    // we be newtoning
    for (int i = 0; i < 20; i++) {
      final double EPSILON = 0.001;
      // get deriv of velocity (dis/time)
      double lowVel =
          (distanceToTarget - EPSILON) / tree.get(distanceToTarget - EPSILON).timeOfFlightSecs();
      double highVel =
          (distanceToTarget + EPSILON) / tree.get(distanceToTarget + EPSILON).timeOfFlightSecs();
      double velDeriv = (highVel - lowVel) / (EPSILON * 2);
      // newtons method: xn+1 = xn - f(xn)/deriv(xn)
      // so estimate for new dist is difference between current velocity required velocity over the
      // deriv
      // this makes sense because if current vel is larger it will lower current distance to account
      // for that and if requird is larger it will increase to account for that
      distanceToTarget -= (distanceToTarget / tof - requiredVelocity) / velDeriv;
      // update
      tof = tree.get(distanceToTarget).timeOfFlightSecs();
    }

    // TODO if we're behind the hub just don't shoot

    Rotation2d turretTarget = target.minus(lookaheadPose.getTranslation()).getAngle();
    turretTarget = getTurretTargetRotation(turretTarget, robotPose);

    return new ShotParams(
        tree.get(distanceToTarget), getTurretTargetRotation(turretTarget, robotPose));
  }

  public static Rotation2d getTurretTargetRotation(
      Rotation2d turretTargetRotation, Pose2d robotPose) {

    // subtract that from rotation to point at target
    turretTargetRotation = turretTargetRotation.minus(robotPose.getRotation());
    Logger.recordOutput("Turret/Unclamped target", turretTargetRotation);
    // -5 is some insane fudge factor i forgot where it's from
    double turretTargetDegrees = turretTargetRotation.getDegrees() - 5;
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

  public static ShotParams getParametersMechA(
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
                (robotRelativeVelocity.vxMetersPerSecond * AutoAim.LATENCY_COMPENSATION_SECS)
                    + (0.5 * axMetersPerSecSq * Math.pow(AutoAim.LATENCY_COMPENSATION_SECS, 2)),
                (robotRelativeVelocity.vyMetersPerSecond * AutoAim.LATENCY_COMPENSATION_SECS)
                    + (0.5 * ayMetersPerSecSq * Math.pow(AutoAim.LATENCY_COMPENSATION_SECS, 2)),
                (robotRelativeVelocity.omegaRadiansPerSecond * AutoAim.LATENCY_COMPENSATION_SECS)
                    + (0.5 * alphaRadPerSecSq * Math.pow(AutoAim.LATENCY_COMPENSATION_SECS, 2))));

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
    Rotation2d turretAngle = AutoAim.getTargetRotation(target, lookaheadPose);
    turretAngle = getTurretTargetRotation(turretAngle, estimatedPose);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return new ShotParams(tree.get(lookaheadTurretToTargetDistance), turretAngle);
  }

  public static boolean targetInTurretDeadzone() {
    return outOfRange;
  }
}
