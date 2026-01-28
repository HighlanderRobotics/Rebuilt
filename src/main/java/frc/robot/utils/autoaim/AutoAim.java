package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import org.littletonrobotics.junction.Logger;

public class AutoAim {

  public static double LATENCY_COMPENSATION_SECS = 0.02; // TODO tune latency comp

  public static final InterpolatingShotTree HUB_SHOT_TREE = new InterpolatingShotTree();

  static { // For hub shot tree
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 + 17),
        new ShotData(Rotation2d.fromDegrees(8), 27.5, 1.46, Units.inchesToMeters(24 + 17) / 1.46));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 12),
        new ShotData(
            Rotation2d.fromDegrees(6),
            30,
            1.55,
            Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 12) / 1.55));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 3 * 12),
        new ShotData(
            Rotation2d.fromDegrees(10.5),
            30,
            1.54,
            Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 3 * 12) / 1.54));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 5 * 12),
        new ShotData(
            Rotation2d.fromDegrees(14.5),
            30,
            1.54,
            Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 5 * 12) / 1.54));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 7 * 12),
        new ShotData(
            Rotation2d.fromDegrees(18),
            30,
            1.52,
            Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 7 * 12) / 1.52));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 9 * 12),
        new ShotData(
            Rotation2d.fromDegrees(21.5),
            30,
            1.46,
            Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 9 * 12) / 1.46));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 11 * 12),
        new ShotData(
            Rotation2d.fromDegrees(24.5),
            30,
            1.35,
            Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 11 * 12) / 1.35));
    HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 13 * 12),
        new ShotData(
            Rotation2d.fromDegrees(28),
            30,
            1.36,
            Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 13 * 12) / 1.36));
  }

  // im sorry kevin

  private static final InterpolatingTreeMap<Double, Rotation2d> velocityHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  static {
    velocityHoodAngleMap.put(Units.inchesToMeters(24 + 17) / 1.46, Rotation2d.fromDegrees(8));
    velocityHoodAngleMap.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 12) / 1.55, Rotation2d.fromDegrees(6));
    velocityHoodAngleMap.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 3 * 12) / 1.54, Rotation2d.fromDegrees(10.5));
    velocityHoodAngleMap.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 5 * 12) / 1.54, Rotation2d.fromDegrees(14.5));
    velocityHoodAngleMap.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 7 * 12) / 1.52, Rotation2d.fromDegrees(18));
    velocityHoodAngleMap.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 9 * 12) / 1.46, Rotation2d.fromDegrees(21.5));
    velocityHoodAngleMap.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 11 * 12) / 1.35, Rotation2d.fromDegrees(24.5));
    velocityHoodAngleMap.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 13 * 12) / 1.36, Rotation2d.fromDegrees(28));
  }

  // Ig we'll see if we need more than 1 feed shot tree
  public static final InterpolatingShotTree FEED_SHOT_TREE = new InterpolatingShotTree();

  static { // For feed shot tree
    // TODO: POPULATE
    FEED_SHOT_TREE.put(
        1.0, new ShotData(Rotation2d.kCW_90deg, 10, 0, 0)); // Placeholder to prevent crashes
  }

  // TODO: SOTM

  public static double distanceToHub(Pose2d pose) {
    double distance = pose.getTranslation().getDistance(FieldUtils.getCurrentHubTranslation());
    Logger.recordOutput("Autoaim/Distance To Hub", distance);
    return distance;
  }

  public static Rotation2d getSOTMYaw(Pose2d robot, ChassisSpeeds fieldRelativeSpeeds) {
    // // V_ball-ground = V_ball-robot + V_robot-ground (relative motion)
    // // if we want the ball to go straight towards the goal,
    // // the V_ball-robot vector needs to cancel out with the V_robot-ground vector to "offset" the
    // // velocity it already has
    // //this is the desired final ground velocity of the ball
    // double v_ballGround =
    //     HUB_SHOT_TREE.calculateShot(robot).groundVelocity();
    // // let phi be the azimuth
    // // phi = arcsin(-V_robot-ground / |V_ball-ground|)
    // double phi = Math.asin((-1) * fieldRelativeSpeeds.vyMetersPerSecond / v_ballGround);
    // return Rotation2d.fromRadians(phi);

    // the ball exits the shooter with velocity v at an angle theta (just assume it's the correct
    // velocity and angle)
    // the magnitude of the V_ball-ground vector (or |V_ball-ground|) is v * cos (theta)
    double v_BallGround = HUB_SHOT_TREE.calculateShot(robot).groundVelocity();

    double v_RobotGround = fieldRelativeSpeeds.vyMetersPerSecond; // sob
    // use law of cosines to find needed velocity of the ball relative to the ground
    // let the current angle between the robot and hub be alpha
    // the following is from the static autoaim
    Translation2d robotHubVec = FieldUtils.getCurrentHubTranslation().minus(robot.getTranslation());
    // return FieldUtils.getCurrentHubPose().minus(getPose()).getRotation();
    // Logger.recordOutput("robot hub vec", robotHubVec);
    // atan2 takes y as the first arg (i think bc θ = atan(y/x) but idk)
    Rotation2d alpha =
        Rotation2d.k180deg.minus(
            Rotation2d.fromRadians(Math.atan2(robotHubVec.getY(), robotHubVec.getX())));
    double v_BallRobot =
        Math.sqrt(
            Math.pow(v_RobotGround, 2)
                + Math.pow(v_BallGround, 2)
                - 2 * v_RobotGround * v_BallGround * alpha.getCos());

    // use law of sines to find heading
    // let phi be the desired angle
    // V_ball-robot / alpha = V_ball_ground / phi
    // so phi = alpha * v_ball-ground / v_ball-robot

    double phi = alpha.getRadians() * v_BallGround / v_BallRobot;
    // let phi be the azimuth
    // phi = arcsin(-V_robot-ground / |V_ball-ground|)
    // double phi = Math.acos((-1) * fieldRelativeSpeeds.vyMetersPerSecond / v_BallGround);
    Logger.recordOutput(
        "sotm target??", robot.transformBy(new Transform2d(0, 5, new Rotation2d())));

    // Logger.recordOutput(
    //     "autoaim target",
    //     new Pose3d(
    //         new Translation3d(
    //                 swerveSimulation
    //                     .getSimulatedDriveTrainPose()
    //                     .getTranslation())
    //             .plus(new Translation3d(0, 0, 1)),
    //         new Rotation3d(
    //             AutoAim.getSOTMHeading(
    //                 getPose(), getVelocityFieldRelative()))));
    if (v_RobotGround < 0.05) return alpha;
    else {
      return robot.getRotation().plus(Rotation2d.fromRadians(phi)).plus(Rotation2d.kCW_90deg);
    }
  }

  // public static Rotation2d getSOTMPitch(Pose2d robot, ChassisSpeeds fieldRelativeSpeeds) {
  // //simple case in which you are moving straight backwards from the goal
  //     //you know your current distance from the goal
  //     //that means you know the ball's ground velocity if you were to launch it right then
  //     double v_BallGround = HUB_SHOT_TREE.calculateShot(robot).groundVelocity();
  //     //you know your current velocity
  //     double v_RobotGround = fieldRelativeSpeeds.vxMetersPerSecond;
  //     double desiredV_BallGround = v_BallGround - v_RobotGround;
  //     ShotData shot = HUB_SHOT_TREE.get(null)

  // }
  // brooooo

  public static Translation2d getBallGroundVector(Pose2d robot) {
    Translation2d robotToHub = FieldUtils.getCurrentHubTranslation().minus(robot.getTranslation());
    double ballGroundVelocity = AutoAim.HUB_SHOT_TREE.calculateShot(robot).groundVelocity();
    double v_x = ballGroundVelocity * robotToHub.getAngle().getCos();
    double v_y =
        ballGroundVelocity * robotToHub.getAngle().getSin() * Math.signum(robotToHub.getY());
    Logger.recordOutput("angle", robotToHub.getAngle());
    Translation2d V_BallGround = new Translation2d(v_x, v_y);
    return V_BallGround;
  }

  public static Rotation2d getSOTMYawfr(Pose2d robot, ChassisSpeeds fieldChassisSpeeds) {
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldChassisSpeeds, robot.getRotation());
    // calculate latency compensated pose
    Pose2d compensatedPose =
        robot.exp(
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * LATENCY_COMPENSATION_SECS,
                robotRelativeSpeeds.vyMetersPerSecond * LATENCY_COMPENSATION_SECS,
                robotRelativeSpeeds.omegaRadiansPerSecond * LATENCY_COMPENSATION_SECS));

    // note that this is a position vector not velocity
    Translation2d robotToHub =
        FieldUtils.getCurrentHubTranslation().minus(compensatedPose.getTranslation());
    // V_ball-ground = V_ball-robot + V_robot-ground (relative motion)
    Translation2d V_RobotGround =
        new Translation2d(
            fieldChassisSpeeds.vxMetersPerSecond, fieldChassisSpeeds.vyMetersPerSecond);
    Translation2d V_BallGround = AutoAim.getBallGroundVector(compensatedPose);
    Translation2d V_BallRobot = V_BallGround.minus(V_RobotGround);

    Rotation2d rot = V_BallRobot.getAngle();
    if (V_RobotGround.getNorm() < 0.15) {

      // return FieldUtils.getCurrentHubPose().minus(getPose()).getRotation();
      // Logger.recordOutput("robot hub vec", robotHubVec);
      // atan2 takes y as the first arg (i think bc θ = atan(y/x) but idk)
      rot = Rotation2d.fromRadians(Math.atan2(robotToHub.getY(), robotToHub.getX()));
    }
    rot = rot.rotateBy(Rotation2d.k180deg);

    Pose2d poseSetpoint = new Pose2d(robot.getTranslation(), rot);
    Logger.recordOutput("what this bastard is supposed to be doing", poseSetpoint);
    Logger.recordOutput(
        "Autoaim/Target viz", poseSetpoint.transformBy(new Transform2d(10, 0, new Rotation2d())));
    Logger.recordOutput("hi we are still alive", Logger.getTimestamp());
    // new Pose2d(V_BallGround.times(-1), rot));
    return rot;
  }

  public static Rotation2d getSOTMPitchfr(Pose2d robot, ChassisSpeeds fieldChassisSpeeds) {

    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldChassisSpeeds, robot.getRotation());
    // calculate latency compensated pose
    Pose2d compensatedPose =
        robot.exp(
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * LATENCY_COMPENSATION_SECS,
                robotRelativeSpeeds.vyMetersPerSecond * LATENCY_COMPENSATION_SECS,
                robotRelativeSpeeds.omegaRadiansPerSecond * LATENCY_COMPENSATION_SECS));

    Translation2d V_BallGround = AutoAim.getBallGroundVector(compensatedPose);
    double groundVelocity = V_BallGround.getNorm();
    Logger.recordOutput("Autoaim/Ground vel", groundVelocity);
    Rotation2d hoodAngle = velocityHoodAngleMap.get(groundVelocity);
    return hoodAngle;
  }
}
