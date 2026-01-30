package frc.robot.subsystems.swerve.constants.comp;

import edu.wpi.first.math.util.Units;

public class R2CompBotSwerveConstants extends R1CompBotSwerveConstants {
    @Override
  public double getMaxLinearSpeed() {
    // From https://www.swervedrivespecialties.com/collections/kits/products/mk5n-swerve-module
    // SDS Mk5n, R2 ratio, no FOC (because FOC is disabled if we're going fast enough)
    return Units.feetToMeters(17.4);
  }

  @Override
  public double getMaxLinearAcceleration() {
    // Calculated in Choreo for R2 ratio
    return 7.768;
  }

  @Override
  public double getDriveGearRatio() {
    // From https://www.swervedrivespecialties.com/collections/kits/products/mk5n-swerve-module
    // Mk5n, R2 ratio
    return 6.03;
  }
}
