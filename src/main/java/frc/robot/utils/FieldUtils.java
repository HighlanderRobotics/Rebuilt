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

/** Add your docs here. */
public class FieldUtils {
  // From eyeballing in choreo
  public static final Pose2d BLUE_HUB_POS =
      new Pose2d(4.686160087585449, 4.030325412750244, Rotation2d.kZero);
  public static final Pose2d RED_HUB_POS = ChoreoAllianceFlipUtil.flip(BLUE_HUB_POS);

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
  }

  public enum ClimbTargets {
    // Grabbed in Choreo. Needs real testing
    // 3.05 is climber offset
    BLUE_RIGHT(new Pose2d(1.189 - Units.inchesToMeters(3.05), 4.658, Rotation2d.kCCW_90deg)),
    BLUE_LEFT(new Pose2d(1.189 - Units.inchesToMeters(3.05) ,2.845, Rotation2d.kCW_90deg)),
    RED_RIGHT(ChoreoAllianceFlipUtil.flip(BLUE_RIGHT.getPose())),
    RED_LEFT(ChoreoAllianceFlipUtil.flip(BLUE_LEFT.getPose()))
    ;

    private Pose2d targetPose;

    private ClimbTargets(Pose2d pose) {
      this.targetPose = pose;
    }

    public Pose2d getPose() {
      return targetPose;
    }
  }
}
