// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Superstructure.FeedTarget;
import java.util.Arrays;
import java.util.List;

/** Add your docs here. */
public class FieldUtils {
  // From eyeballing in choreo
  public static final Pose2d BLUE_HUB_POS =
      new Pose2d(4.686160087585449, 4.030325412750244, Rotation2d.kZero);
  public static final Pose2d RED_HUB_POS = ChoreoAllianceFlipUtil.flip(BLUE_HUB_POS);

  public static final Pose2d BLUE_BUMP_RIGHT_POS = new Pose2d(4.62, 2.505, Rotation2d.kZero);

  public static final Pose2d BLUE_BUMP_LEFT_POS = new Pose2d(4.62, 5.555, Rotation2d.kZero);

  public static final Pose2d RED_BUMP_RIGHT_POS = new Pose2d(11.91, 2.505, Rotation2d.kZero);

  public static final Pose2d RED_BUMP_LEFT_POS = new Pose2d(11.91, 5.555, Rotation2d.kZero);

  public static Translation2d getCurrentHubTranslation() {
    return getCurrentHubPose().getTranslation();
  }

  public static Pose2d getCurrentHubPose() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? BLUE_HUB_POS
        : RED_HUB_POS;
  }

  public enum FeedTargets {
    BLUE_BACK_RIGHT(new Translation2d(0.6643, 0.75)), // Eyeballed in Choreo
    BLUE_BACK_LEFT(new Translation2d(0.75, 7.367)),
    RED_BACK_RIGHT(ChoreoAllianceFlipUtil.flip(BLUE_BACK_RIGHT.getPose())),
    RED_BACK_LEFT(ChoreoAllianceFlipUtil.flip(BLUE_BACK_LEFT.getPose()));

    private Pose2d targetPose;

    private FeedTargets(Pose2d pose) {
      this.targetPose = pose;
    }

    private FeedTargets(Translation2d translation) {
      this.targetPose = new Pose2d(translation, Rotation2d.kZero);
    }

    public Pose2d getPose() {
      return targetPose;
    }

    public Translation2d getTranslation() {
      return targetPose.getTranslation();
    }

    public static FeedTargets getFeedTarget(FeedTarget target) {
      if (target == FeedTarget.LEFT) {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? BLUE_BACK_LEFT
            : RED_BACK_LEFT;
      } else {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? BLUE_BACK_RIGHT
            : RED_BACK_RIGHT;
      }
    }
  }

  public enum ClimbTargets {
    // Grabbed in Choreo. Needs real testing
    // 3.05 is climber offset
    BLUE_LEFT(
        new Pose2d(
            1.189 - Units.inchesToMeters(3.05),
            4.658 - Units.inchesToMeters(1),
            Rotation2d.kCCW_90deg),
        true,
        true),
    BLUE_RIGHT(
        // x2 offset because we need to offset again after rotating 180 deg
        new Pose2d(
            1.189 - (Units.inchesToMeters(3.05) * 2),
            2.845 + Units.inchesToMeters(1),
            Rotation2d.kCW_90deg),
        false,
        true),
    RED_RIGHT(ChoreoAllianceFlipUtil.flip(BLUE_RIGHT.getPose()), false, false),
    RED_LEFT(ChoreoAllianceFlipUtil.flip(BLUE_LEFT.getPose()), true, false);

    private Pose2d targetPose;
    private boolean leftHanded;
    private boolean isBlueAlliance;

    private ClimbTargets(Pose2d pose, boolean leftHanded, boolean isBlueAlliance) {
      this.targetPose = pose;
      this.leftHanded = leftHanded;
      this.isBlueAlliance = isBlueAlliance;
    }

    public Pose2d getPose() {
      return targetPose;
    }

    public boolean getLeftHanded() {
      return leftHanded;
    }

    public boolean isBlueAlliance() {
      return isBlueAlliance;
    }

    public static final List<ClimbTargets> CLIMB_TARGETS_LIST =
        Arrays.stream(ClimbTargets.values()).toList();
  }

  public enum TrenchPoses {
    BLUE_RIGHT(new Pose2d(4.704, 0.741, Rotation2d.kZero)),
    BLUE_LEFT(new Pose2d(4.704, 7.34, Rotation2d.kZero)),
    RED_LEFT(ChoreoAllianceFlipUtil.flip(BLUE_LEFT.getPose())),
    RED_RIGHT(ChoreoAllianceFlipUtil.flip(BLUE_RIGHT.getPose()));

    private Pose2d pose;

    private TrenchPoses(Pose2d pose) {
      this.pose = pose;
    }

    public Pose2d getPose() {
      return pose;
    }

    public static final List<TrenchPoses> TRENCH_POSES_LIST =
        Arrays.stream(TrenchPoses.values()).toList();

    public static Pose2d getClosestTrenchPose(Pose2d pose) {
      double minDistance = Double.MAX_VALUE;
      Pose2d trench = Pose2d.kZero;
      for (TrenchPoses t : TRENCH_POSES_LIST) {
        if (Math.abs(t.pose.minus(pose).getTranslation().getNorm()) < minDistance) {
          trench = t.pose;
        }
      }
      return trench;
    }
  }
}
