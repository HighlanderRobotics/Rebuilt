package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.utils.autoaim.AutoAim.ShotParams;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;

public class ShotTrees {
  public static final InterpolatingShotTree ALPHA_HUB_SHOT_TREE = new InterpolatingShotTree();

  static { // For hub shot tree
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 + 17), new ShotData(Rotation2d.fromDegrees(8), 27.5, 1.46));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 12),
        new ShotData(Rotation2d.fromDegrees(6), 30, 1.55));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 3 * 12),
        new ShotData(Rotation2d.fromDegrees(10.5), 30, 1.54));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 5 * 12),
        new ShotData(Rotation2d.fromDegrees(14.5), 30, 1.54));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 7 * 12),
        new ShotData(Rotation2d.fromDegrees(18.25), 30, 1.52));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 9 * 12),
        new ShotData(Rotation2d.fromDegrees(21.5), 30, 1.46));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 11 * 12),
        new ShotData(Rotation2d.fromDegrees(24.5), 30, 1.35));
    ALPHA_HUB_SHOT_TREE.put(
        Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 13 * 12),
        new ShotData(Rotation2d.fromDegrees(28), 30, 1.36));
  }

  public static final InterpolatingShotTree COMP_HUB_SHOT_TREE = new InterpolatingShotTree();

  static {
    COMP_HUB_SHOT_TREE.put(
        1.716849,
        new ShotData(
            Rotation2d.fromDegrees(23 - 13.16),
            30 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            0.8));
    COMP_HUB_SHOT_TREE.put(
        2.017596,
        new ShotData(
            Rotation2d.fromDegrees(23 - 13.16),
            33 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            0.9));
    COMP_HUB_SHOT_TREE.put(
        2.423868,
        new ShotData(
            Rotation2d.fromDegrees(25 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.1));
    COMP_HUB_SHOT_TREE.put(
        2.664198,
        new ShotData(
            Rotation2d.fromDegrees(26 - 13.16),
            36 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2));
    COMP_HUB_SHOT_TREE.put(
        2.903207,
        new ShotData(
            Rotation2d.fromDegrees(30 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2));
    COMP_HUB_SHOT_TREE.put(
        3.156802,
        new ShotData(
            Rotation2d.fromDegrees(32 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.23));
    COMP_HUB_SHOT_TREE.put(
        3.437033,
        new ShotData(
            Rotation2d.fromDegrees(34 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.25));
    COMP_HUB_SHOT_TREE.put(
        3.611052,
        new ShotData(
            Rotation2d.fromDegrees(38 - 13.16),
            34 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.24));
    COMP_HUB_SHOT_TREE.put(
        3.773999,
        new ShotData(
            Rotation2d.fromDegrees(39 - 13.16),
            34 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.21));
    COMP_HUB_SHOT_TREE.put(
        3.899275,
        new ShotData(
            Rotation2d.fromDegrees(40 - 13.16),
            34 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2));
    COMP_HUB_SHOT_TREE.put(
        4.138058,
        new ShotData(
            Rotation2d.fromDegrees(41 - 13.16),
            34 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.13));

    // budget shots
    // COMP_HUB_SHOT_TREE.put(4.172776, new ShotData(Rotation2d.fromDegrees(36), 25.5, 1.13));

    // COMP_HUB_SHOT_TREE.put(4.512483, new ShotData(Rotation2d.fromDegrees(34), 25.5, 1.2));

    // COMP_HUB_SHOT_TREE.put(4.974656, new ShotData(Rotation2d.fromDegrees(39), 27, 1.2));

    COMP_HUB_SHOT_TREE.put(
        4.602258,
        new ShotData(
            Rotation2d.fromDegrees(43 - 13.16),
            34.5 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.15));
    COMP_HUB_SHOT_TREE.put(
        4.893493,
        new ShotData(
            Rotation2d.fromDegrees(45 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2));
    COMP_HUB_SHOT_TREE.put(
        5.225402,
        new ShotData(
            Rotation2d.fromDegrees(47 - 13.16),
            35 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2));
    COMP_HUB_SHOT_TREE.put(
        5.584793,
        new ShotData(
            Rotation2d.fromDegrees(49 - 13.16),
            35  * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.17));
  }

  public static final InterpolatingShotTree FEED_SHOT_TREE = new InterpolatingShotTree();

  static { // For feed shot tree
    FEED_SHOT_TREE.put(
        Units.feetToMeters(2), new ShotData(Rotation2d.fromDegrees(23.16), 20 - 2, 0)); // - 2, 0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(4),
        new ShotData(Rotation2d.fromDegrees(30), 40 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(6),
        new ShotData(Rotation2d.fromDegrees(40), 30 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(8),
        new ShotData(Rotation2d.fromDegrees(40), 32 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(10),
        new ShotData(Rotation2d.fromDegrees(40), 35 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(12),
        new ShotData(Rotation2d.fromDegrees(40), 40 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(14),
        new ShotData(Rotation2d.fromDegrees(45), 38 - 2, 0.0)); // - 2, 0.0));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(16),
        new ShotData(Rotation2d.fromDegrees(45), 40 - 2, 0.0)); // - 2, 0.0));

    FEED_SHOT_TREE.put(
        Units.feetToMeters(18),
        new ShotData(
            Rotation2d.fromDegrees(40 - 13.16),
            36 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.42)); // - 2, 1.42));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(20),
        new ShotData(
            Rotation2d.fromDegrees(43 - 13.16),
            38 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.36)); // - 2, 1.36));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(22),
        new ShotData(
            Rotation2d.fromDegrees(45 - 13.16),
            39 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.34)); // - 2, 1.34));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(24),
        new ShotData(
            Rotation2d.fromDegrees(47 - 13.16),
            40 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.25)); // - 2, 1.25));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(26),
        new ShotData(
            Rotation2d.fromDegrees(48 - 13.16),
            41 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.28)); // - 2, 1.28));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(28),
        new ShotData(
            Rotation2d.fromDegrees(49 - 13.16),
            43 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.27)); // - 2, 1.27));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(30),
        new ShotData(
            Rotation2d.fromDegrees(49 - 13.16),
            44 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.32)); // - 2, 1.32));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(32),
        new ShotData(
            Rotation2d.fromDegrees(49 - 13.16),
            46 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.4)); // - 2, 1.4));

    FEED_SHOT_TREE.put(
        Units.feetToMeters(34),
        new ShotData(
            Rotation2d.fromDegrees(52 - 13.16),
            47 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.3)); // - 2, 1.3));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(36),
        new ShotData(
            Rotation2d.fromDegrees(53 - 13.16),
            51 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.33)); // - 2, 1.33));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(38),
        new ShotData(
            Rotation2d.fromDegrees(53 - 13.16),
            55 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.3)); // - 2, 1.3));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(40),
        new ShotData(
            Rotation2d.fromDegrees(55 - 13.16),
            55 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2)); // - 2, 1.2));
    FEED_SHOT_TREE.put(
        Units.feetToMeters(42),
        new ShotData(
            Rotation2d.fromDegrees(56 - 13.16),
            57 * 0.84615384615 / TurretSubsystem.FLYWHEEL_GEAR_RATIO + 1,
            1.2)); // - 2, 1.2));
  }

  // Not really using these but ig i'll keep them
  public static ShotParams getLeftFixedShotParams() {
    return new ShotParams(
        new ShotData(Rotation2d.fromDegrees(36), 36, 0), Rotation2d.fromDegrees(-73.916016));
  }

  public static ShotParams getRightFixedShotParams() {
    return new ShotParams(
        new ShotData(Rotation2d.fromDegrees(23.16), 35.7, 0), Rotation2d.fromDegrees(-82));
  }

  public static ShotParams getMidFixedShotParams() {
    return new ShotParams(
        new ShotData(Rotation2d.fromDegrees(32.84), 35, 0), Rotation2d.fromDegrees(-109.775391));
  }
}
