package frc.robot.subsystems.swerve.constants.comp;

import edu.wpi.first.math.util.Units;

public class R3WispSwerveConstants extends R1WispSwerveConstants {
  @Override
  public double getMaxLinearSpeed() {
    // From https://www.swervedrivespecialties.com/collections/kits/products/mk5n-swerve-module
    // SDS Mk5n, R3 ratio, no FOC (because FOC is disabled if we're going fast enough)
    return Units.feetToMeters(19.9);
  }

  @Override
  public double getMaxLinearAcceleration() {
    // Calculated in Choreo for R3 ratio
    return 6.789;
  }

  @Override
  public double getDriveGearRatio() {
    // From https://www.swervedrivespecialties.com/collections/kits/products/mk5n-swerve-module
    // Mk5n, R3 ratio
    return 5.27;
  }
}
