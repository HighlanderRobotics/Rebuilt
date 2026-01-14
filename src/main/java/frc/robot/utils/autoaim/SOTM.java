// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class SOTM {
    public Pose3d getVirtualTarget(Pose3d target, ChassisSpeeds fieldRelativeSpeeds, double shotTime) {
        // velocity times shot time is how translated it is
        return target.transformBy(
        new Transform3d(
                fieldRelativeSpeeds.vxMetersPerSecond
                    * shotTime,
                fieldRelativeSpeeds.vyMetersPerSecond
                    * shotTime,
                0,
                new Rotation3d())
            .inverse());
    }
}
