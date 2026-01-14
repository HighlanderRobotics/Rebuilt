// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldUtils {
  // From eyeballing in choreo
  public static final Translation2d BLUE_HUB_POS =
      new Translation2d(4.686160087585449, 4.030325412750244);
  public static final Translation2d RED_HUB_POS = ChoreoAllianceFlipUtil.flip(BLUE_HUB_POS);

  public static Translation2d getCurrentHubPos() {
    if (DriverStation.getAlliance().isEmpty()) return BLUE_HUB_POS;
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return BLUE_HUB_POS;
    } else {
      return RED_HUB_POS;
    }
  }
}
