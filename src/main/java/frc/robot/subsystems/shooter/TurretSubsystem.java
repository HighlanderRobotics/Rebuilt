// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.Robot.RobotEdition;
import frc.robot.Superstructure;
import frc.robot.components.cancoder.CANcoderIO;
import frc.robot.components.cancoder.CANcoderIOInputsAutoLogged;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FuelSim;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.NewAutoAim;
import frc.robot.utils.autoaim.NewAutoAim.ShotParams;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Pivoting hooded shooter (turret). !! COMP !! */
public class TurretSubsystem extends SubsystemBase implements Shooter {

  /** Creates a new TurretSubsystem. */
  public static final double HOOD_GEAR_RATIO = 33.8671875; // 58.96875;

  public static final double FLYWHEEL_GEAR_RATIO = 20.0 / 18.0; // 0.84615384615;

  public static final Rotation2d HOOD_MAX_ANGLE = Rotation2d.fromDegrees(56);
  public static final Rotation2d HOOD_MIN_ANGLE = Rotation2d.fromDegrees(11.33);
  public static final double HOOD_CURRENT_ZERO_THRESHOLD = 30.0;
  public static final double TURRET_CURRENT_ZERO_THRESHOLD = 30.0; // TODO find

  public static final double FLYWHEEL_DIAMETER_INCHES = 4.0;

  public static final Rotation2d TURRET_LEFT_FIXED_SHOT_ANGLE = Rotation2d.kZero;
  public static final Rotation2d TURRET_RIGHT_FIXED_SHOT_ANGLE = Rotation2d.kZero;
  public static final Rotation2d TURRET_MID_FIXED_SHOT_ANGLE = Rotation2d.kZero;

  public static final double TURRET_GEAR_RATIO = (42.0 / 12.0) * (32.0 / 16.0) * (85.0 / 10.0);

  public static final double CANCODER_24T_TO_TURRET_GEAR_RATIO = (24.0 / 32.0) * (10.0 / 85.0);
  public static final double CANCODER_26T_TO_TURRET_GEAR_RATIO = (26.0 / 32.0) * (10.0 / 85.0);

  // TODO: REDO THIS HARDSTOP WHEN FIXED??
  // logged for ease of graph viewing
  @AutoLogOutput(key = "Shooter/Turret/Left Hardstop")
  public static final Rotation2d TURRET_LEFT_HARDSTOP_ANGLE =
      // Changed to avoid cooking cable chain/wires
      // Plus 0 because then the rotation2d automatically wraps the value between -0.5 and 0.5
      // (worked in sim)
      Rotation2d.fromDegrees(-259.013672).plus(Rotation2d.kZero); // 0.25 // -0.75 // -0.719536);

  @AutoLogOutput(key = "Shooter/Turret/Forward Hardstop")
  public static Rotation2d TURRET_FORWARD_HARDSTOP_ANGLE =
      Rotation2d.fromDegrees(
          -10); // -0.0354 // 0.011378); //slightly short of what it actually is (0.002 ish) but

  // otherwise wrapping gets weird

  public static final Translation2d ROBOT_TO_TURRET_TRANSLATION =
      new Translation2d(-0.177413, -0.111702); // , 0.350341);
  public static final double FLYWHEEL_VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND = 5.0;

  private double hoodCurrentFilterValue = 0.0;
  private double turretCurrentFilterValue = 0.0;

  private CANcoderIO cancoder24t;
  private CANcoderIO cancoder26t;
  private CANcoderIOInputsAutoLogged cancoder24tInputs = new CANcoderIOInputsAutoLogged();
  private CANcoderIOInputsAutoLogged cancoder26tInputs = new CANcoderIOInputsAutoLogged();

  private HoodIO hoodIO;
  private HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private FlywheelIO flywheelIO;
  private FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private TurretIO turretIO;
  private TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  private LinearFilter currentFilter = LinearFilter.movingAverage(10);

  private SysIdRoutine hoodSysid =
      new SysIdRoutine(
          new Config(
              null,
              Volts.of(5),
              null,
              (state) -> Logger.recordOutput("Shooter/Hood/SysID State", state.toString())),
          new Mechanism((voltage) -> hoodIO.setHoodVoltage(voltage.in(Volts)), null, this));

  private SysIdRoutine flywheelSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Shooter/Flywheel/SysID State", state.toString())),
          new Mechanism((voltage) -> flywheelIO.setFlywheelVoltage(voltage.in(Volts)), null, this));

  private static final Alert cancoder24tDisconnectedAlert =
      new Alert("24T Cancoder disconnected!", AlertType.kError);
  private static final Alert cancoder26tDisconnectedAlert =
      new Alert("26T Cancoder disconnected!", AlertType.kError);
  private final Alert flywheelLeaderDisconnectedAlert =
      new Alert("Disconnected flywheel leader!", AlertType.kError);
  private final Alert flywheelFollowerDisconnectedAlert =
      new Alert("Disconnected flywheel follower!", AlertType.kError);
  private final Alert hoodDisconnectedAlert = new Alert("Disconnected hood!", AlertType.kError);
  private final Alert turretMotorDisconnectedAlert =
      new Alert("Disconnected turret motor!", AlertType.kError);

  private final Alert turretPastHardstopAlert =
      new Alert(
          "Turret may have gone past hardstop!! Reoffset cancoders + min/max position",
          AlertType.kError);

  private FuelSim fuelSim;

  public TurretSubsystem(
      FlywheelIO flywheelIO,
      HoodIO hoodIO,
      TurretIO turretIO,
      CANcoderIO cancoder24t,
      CANcoderIO cancoder26t,
      FuelSim fuelSim) {
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
    this.turretIO = turretIO;
    this.cancoder24t = cancoder24t;
    this.cancoder26t = cancoder26t;
    this.fuelSim = fuelSim;

    // assume we start up at min angle and not 0
    hoodIO.resetEncoder(HOOD_MIN_ANGLE);
  }

  @Override
  public void turretInit() {
    // Starting positions
    this.cancoder24t.updateInputs(cancoder24tInputs);
    this.cancoder26t.updateInputs(cancoder26tInputs);

    Logger.processInputs("Shooter/Turret Cancoder24t", cancoder24tInputs);
    Logger.processInputs("Shooter/Turret Cancoder26t", cancoder26tInputs);

    // Calculate the start angle based on the two encoders
    Rotation2d startAngle = getCalculatedTurretRotations();
    Logger.recordOutput("Shooter/Turret/Starting Angle", startAngle);
    // Set the sensor position to the start angle
    turretIO.resetTurretEncoder(startAngle);
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("Shooter/Turret", turretInputs);
    cancoder24t.updateInputs(cancoder24tInputs);
    Logger.processInputs("Shooter/Turret Cancoder24t", cancoder24tInputs);
    cancoder26t.updateInputs(cancoder26tInputs);
    Logger.processInputs("Shooter/Turret Cancoder26t", cancoder26tInputs);
    hoodCurrentFilterValue = currentFilter.calculate(hoodInputs.hoodStatorCurrentAmps);

    cancoder24tDisconnectedAlert.set(!cancoder24tInputs.connected);
    cancoder26tDisconnectedAlert.set(!cancoder26tInputs.connected);
    flywheelLeaderDisconnectedAlert.set(!flywheelInputs.flywheelLeaderConnected);
    flywheelFollowerDisconnectedAlert.set(!flywheelInputs.flywheelFollowerConnected);
    hoodDisconnectedAlert.set(!hoodInputs.connected);
    turretMotorDisconnectedAlert.set(!turretInputs.connected);
    // boolean pastHardstop =
    //     ((getTurretPosition().getDegrees() > 0.002 * 360)
    //             // TurretSubsystem.TURRET_FORWARD_HARDSTOP_ANGLE.getDegrees())
    //             && (getTurretPosition().getDegrees()
    //                 < TurretSubsystem.TURRET_REAR_HARDSTOP_ANGLE.getDegrees())
    //         || (getCalculatedTurretRotations().getDegrees() > 0.002 * 360)
    //             // TurretSubsystem.TURRET_FORWARD_HARDSTOP_ANGLE.getDegrees())
    //             && (getCalculatedTurretRotations().getDegrees()
    //                 < TurretSubsystem.TURRET_REAR_HARDSTOP_ANGLE.getDegrees()));
    // if (pastHardstop) turretPastHardstopAlert.set(pastHardstop); // sticky alert

    if (Superstructure.getState().isAScoreState() && Robot.isSimulation()) {
      System.out.println("launching fuel");
      fuelSim.launchFuel(
          // there are few things i despise more than the units library\
          // InchesPerSecond.of(
          //     flywheelIO.getSetpointRotPerSec() * FLYWHEEL_DIAMETER_INCHES * Math.PI),
          // InchesPerSecond.of(200),
          angularToLinearVelocity(
              RotationsPerSecond.of(flywheelIO.getSetpointRotPerSec()),
              Inches.of(FLYWHEEL_DIAMETER_INCHES / 2)),
          Rotation2d.fromDegrees(90).minus(hoodIO.getHoodSetpoint()).getMeasure(),
          turretIO.getTurretSetpoint().getMeasure(),
          Inches.of(13.75));
    }
  }

  public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
    return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters) * 0.54 + 1);
  }

  @Override
  public void simulationPeriodic() {}

  @Override
  public Command feed(
      Supplier<ShotParams> shotParamsSupplier,
      Supplier<Pose2d> feedTarget,
      Supplier<Pose2d> robotPoseSupplier) {
    return resetTurretToCalculatedPosition()
        .andThen(
            this.run(
                () -> {
                  Logger.recordOutput("Robot/Feed Target", feedTarget.get());
                  hoodIO.setHoodPosition(shotParamsSupplier.get().shotData().hoodAngle());
                  // flywheelIO.setTorqueCurrentVel(shotDataSupplier.get().flywheelVelocityRotPerSec());
                  flywheelIO.setMotionProfiledFlywheelVelocity(
                      shotParamsSupplier.get().shotData().flywheelVelocityRotPerSec());
                  turretIO.setTurretPosition(shotParamsSupplier.get().turretAngle());
                }));
  }

  @Override
  public Command rest(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplierRobotRel,
      BooleanSupplier inScoringArea,
      Supplier<Pose2d> feedTarget) {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(HOOD_MIN_ANGLE);
          flywheelIO.setFlywheelVoltage(0.0);
          // if (canScore.getAsBoolean()) {
          //   turretIO.setTurretPosition(
          //       AutoAim.getTurretFeedTargetRotation(
          //           FeedTargets.getFeedTarget(Superstructure.getFeedTarget()).getTranslation(),
          //           robotPoseSupplier.get(),
          //           chassisSpeedsSupplier.get()));
          // } else {
          if (inScoringArea.getAsBoolean()) {
            turretIO.setTurretPosition(
                NewAutoAim.getParametersMechA(
                        robotPoseSupplier.get(),
                        chassisSpeedsSupplierRobotRel.get(),
                        FieldUtils.getCurrentHubTranslation(),
                        Robot.ROBOT_EDITION == RobotEdition.ALPHA
                            ? AutoAim.ALPHA_HUB_SHOT_TREE
                            : AutoAim.COMP_HUB_SHOT_TREE)
                    .turretAngle());
            // turretIO.setTurretPosition(
            //     AutoAim.getTurretHubTargetRotation(
            //         FieldUtils.getCurrentHubTranslation(),
            //         robotPoseSupplier.get(),
            //         chassisSpeedsSupplier.get()));
          } else {
            turretIO.setTurretPosition(
                AutoAim.getTurretFeedTargetRotation(
                    feedTarget.get().getTranslation(),
                    robotPoseSupplier.get(),
                    chassisSpeedsSupplierRobotRel.get()));
          }
          // }
        });
  }

  @Override
  public Command spit() {
    return this.run(
        () -> {
          hoodIO.setHoodPosition(HOOD_MIN_ANGLE);
          flywheelIO.setMotionProfiledFlywheelVelocity(30);
          // i think we want it to eject as far out from the robot as possible
          turretIO.setTurretPosition(Rotation2d.fromRotations(-0.5));
        }); // TODO: TUNE HOOD POS AND FLYWHEEL VELOCITY
  }

  @Override
  public Command zeroHood() {
    return this.runOnce(() -> hoodIO.resetEncoder(HOOD_MIN_ANGLE));
  }

  public Command runHoodCurrentZeroing() {
    return this.run(() -> hoodIO.setHoodVoltage(-3.0))
        .until(
            new Trigger(() -> Math.abs(hoodCurrentFilterValue) > HOOD_CURRENT_ZERO_THRESHOLD)
                .debounce(0.25))
        .andThen(Commands.parallel(Commands.print("Hood Zeroed"), zeroHood()));
  }

  public Command score(Supplier<ShotParams> shotParamsSupplier) {
    return resetTurretToCalculatedPosition()
        .andThen(
            this.run(
                () -> {
                  // switch (Superstructure.getFixedShotTarget()) {
                  //     // in front of left trench with intake facing trench
                  //   case LEFT:
                  //     hoodIO.setHoodPosition(AutoAim.getLeftFixedShotData().hoodAngle());
                  //     flywheelIO.setMotionProfiledFlywheelVelocity(
                  //         AutoAim.getLeftFixedShotData().flywheelVelocityRotPerSec());
                  //     turretIO.setTurretPosition(AutoAim.LEFT_FIXED_SHOT_TURRET_ANGLE);
                  //     // in front of tower with intake facing left (to avoid deadzone)
                  //   case MID:
                  //     hoodIO.setHoodPosition(AutoAim.getMidFixedShotData().hoodAngle());
                  //     flywheelIO.setMotionProfiledFlywheelVelocity(
                  //         AutoAim.getMidFixedShotData().flywheelVelocityRotPerSec());
                  //     turretIO.setTurretPosition(AutoAim.MID_FIXED_SHOT_TURRET_ANGLE);
                  //     // in front of right trench with intake facing alliance wall
                  //   case RIGHT:
                  //     hoodIO.setHoodPosition(AutoAim.getRightFixedShotData().hoodAngle());
                  //     flywheelIO.setMotionProfiledFlywheelVelocity(
                  //         AutoAim.getRightFixedShotData().flywheelVelocityRotPerSec());
                  //     turretIO.setTurretPosition(AutoAim.RIGHT_FIXED_SHOT_TURRET_ANGLE);
                  //   case NONE:
                  hoodIO.setHoodPosition(shotParamsSupplier.get().shotData().hoodAngle());
                  flywheelIO.setMotionProfiledFlywheelVelocity(
                      shotParamsSupplier.get().shotData().flywheelVelocityRotPerSec());
                  turretIO.setTurretPosition(shotParamsSupplier.get().turretAngle());
                  // }
                }));
  }

  @Override
  public Command resetTurretToPosition(Rotation2d rot) {
    return this.runOnce(() -> turretIO.resetTurretEncoder(getCalculatedTurretRotations()));
  }

  /** sets the motor encoder to the position calculated from the encoders */
  public Command resetTurretToCalculatedPosition() {
    return Commands.print("Rezeroing turret")
        .andThen(resetTurretToPosition(getCalculatedTurretRotations()));
  }

  @Override
  public Command currentZeroTurretAgainstForwardHardstop() {
    return this.run(() -> turretIO.setVoltage(1.0))
        .until(
            new Trigger(() -> Math.abs(turretCurrentFilterValue) > TURRET_CURRENT_ZERO_THRESHOLD)
                .debounce(0.25))
        .andThen(Commands.parallel(Commands.print("Turret Zeroed"), zeroTurretForwardHardstop()));
  }

  public Command zeroTurretForwardHardstop() {
    return this.runOnce(() -> turretIO.resetTurretEncoder(TURRET_FORWARD_HARDSTOP_ANGLE));
  }

  // for defense and stuff
  @Override
  public Command stopTurret() {
    return this.run(
        () -> {
          turretIO.setVoltage(0);
          flywheelIO.setFlywheelVoltage(0);
          hoodIO.setHoodVoltage(0);
        });
  }

  @Override
  @AutoLogOutput(key = "Shooter/Turret/Turret Calculated Rotation")
  public Rotation2d getCalculatedTurretRotations() {
    // give valaues between 0 and 1
    Rotation2d cancoder24t = cancoder24tInputs.cancoderPositionRotations;
    Rotation2d cancoder26t = cancoder26tInputs.cancoderPositionRotations;

    // if can one is bigger than can 2 its simply can1-can2
    // otherwise can1 + 1 - can2 because we want how much behind can1 it is
    // double diffRotations = (cancoder24t.getRotations() - cancoder26t.getRotations()) % 1;
    double diffRotations =
        // MathUtil.applyDeadband(cancoder24t.getRotations() - cancoder26t.getRotations(), 0.01) > 0
        cancoder24t.getRotations() > cancoder26t.getRotations()
            ? cancoder24t.getRotations() - cancoder26t.getRotations()
            : cancoder24t.getRotations() + 1 - cancoder26t.getRotations();
    Logger.recordOutput(
        "Turret/Uncorrected diff rotations",
        cancoder24t.getRotations() - cancoder26t.getRotations());
    Logger.recordOutput("Turret/Diff Rotations", diffRotations);
    // TODO java seems to not know how mod works
    // keeping track of how many total rots can1 is doing using the diff with can2
    // 26/2 because gear difference of 2
    double absoluteRotationsCan1 = diffRotations * (13.0);
    Logger.recordOutput("Turret/Absolute Rotations Can 24t", absoluteRotationsCan1);

    // turret maxes out at less then 1 rotation which is like 11 can1 rotations anyways and it
    // should work up to there
    // multiply abs can1 rots by the gear ratio
    double turretRotations =
        absoluteRotationsCan1 * TurretSubsystem.CANCODER_24T_TO_TURRET_GEAR_RATIO;
    turretRotations = MathUtil.inputModulus(turretRotations + 0.1, 0, 1);

    // offset such that the shooter facing forward is 0
    return Rotation2d.fromRotations(turretRotations - (325.58 / 360.0));
  }

  @Override
  @AutoLogOutput(key = "Shooter/At Vel Setpoint?")
  public boolean atFlywheelVelocitySetpoint() {
    return MathUtil.isNear(
        flywheelInputs.flywheelLeaderVelocityRotationsPerSecond,
        flywheelIO.getSetpointRotPerSec(),
        flywheelIO.getSetpointRotPerSec() * 0.3);
    // return MathUtil.isNear(
    //     flywheelInputs.flywheelLeaderVelocityRotationsPerSecond,
    //     flywheelIO.getSetpointRotPerSec(),
    //     FLYWHEEL_VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND);
  }

  @Override
  @AutoLogOutput(key = "Shooter/Hood/At Setpoint?")
  public boolean atHoodSetpoint() {
    return MathUtil.isNear(
        hoodInputs.hoodPositionRotations.getDegrees(), hoodIO.getHoodSetpoint().getDegrees(), 1);
  }

  @Override
  @AutoLogOutput(key = "Shooter/Turret/At Setpoint?")
  public boolean atTurretSetpoint() {
    return MathUtil.isNear(
        turretInputs.positionRotations.getDegrees(), getTurretSetpoint().getDegrees(), 1);
  }

  @Override
  public Rotation2d getHoodPosition() {
    return hoodInputs.hoodPositionRotations;
  }

  @Override
  @AutoLogOutput(key = "Shooter/Turret/Setpoint")
  public Rotation2d getTurretSetpoint() {
    return turretIO.getTurretSetpoint();
  }

  @Override
  public Rotation2d getTurretPosition() {
    return turretInputs.positionRotations;
  }

  @AutoLogOutput(key = "Shooter/Turret/Cancoder 24t position")
  public Rotation2d getTurretCancoder24tPosition() {
    return cancoder24tInputs.cancoderPositionRotations;
  }

  @AutoLogOutput(key = "Shooter/Turret/Cancoder 26t position")
  public Rotation2d getTurretCancoder26tPosition() {
    return cancoder26tInputs.cancoderPositionRotations;
  }

  public Command runCurrentZeroing() {
    return this.run(() -> hoodIO.setHoodVoltage(-3.0))
        .until(
            new Trigger(() -> Math.abs(hoodCurrentFilterValue) > HOOD_CURRENT_ZERO_THRESHOLD)
                .debounce(0.25))
        .andThen(Commands.parallel(Commands.print("Hood Zeroed"), zeroHood()));
  }

  @Override
  public Rotation2d getHoodSetpoint() {
    return hoodIO.getHoodSetpoint();
  }

  @Override
  public Pose2d getTurretPose(Pose2d robotPose) {
    return robotPose.transformBy(new Transform2d(ROBOT_TO_TURRET_TRANSLATION, Rotation2d.kZero));
  }

  public static TalonFXConfiguration getFlywheelConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = TurretSubsystem.FLYWHEEL_GEAR_RATIO;

    // slot 0 is for motion profiled velocity
    config.Slot0.kS = 0.33706; // 0.63933;
    config.Slot0.kV = 0.13893; // 0.11582;
    config.Slot0.kA = 0.030026; // 0.020809;
    config.Slot0.kP = 0.4;
    config.Slot0.kD = 0;

    // slot 1 is for torque current
    config.Slot1.kS = 13.0;
    config.Slot1.kV = 0.8;
    // config.Slot1.kA = 0.016433;
    config.Slot1.kP = 3.5;
    config.Slot1.kD = 0.15;

    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = false; // TODO add current limits back!!!
    config.CurrentLimits.SupplyCurrentLimit = 40.0;

    config.MotionMagic.MotionMagicAcceleration = 100.0;

    return config;
  }

  public static TalonFXConfiguration getHoodConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = TurretSubsystem.HOOD_GEAR_RATIO;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kS = 0.57613;
    config.Slot0.kG = 0.55748;
    config.Slot0.kV = 5.4081;
    config.Slot0.kA = 0.14829;
    config.Slot0.kP = 260.0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;

    return config;
  }

  public static TalonFXConfiguration getTurretConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = TurretSubsystem.TURRET_GEAR_RATIO;
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kS = 0.45;
    config.Slot0.kV = 5.7;
    config.Slot0.kP = 240.0;

    config.MotionMagic.MotionMagicAcceleration = 10; // 2.064;
    config.MotionMagic.MotionMagicCruiseVelocity = 20; // 8.0;

    return config;
  }

  public static CANcoderConfiguration getCancoder24tConfigs() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // this is to offset the position where both cancoders are equal to be inside the deadzone
    // Offset measured at rear hardstop (approx -259 degrees)
    config.MagnetSensor.MagnetOffset =
        -0.765137 - TurretSubsystem.CANCODER_24T_TO_TURRET_GEAR_RATIO / 0.1;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;

    return config;
  }

  public static CANcoderConfiguration getCancoder26tConfigs() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // this is to offset the position where both cancoders are equal to be inside the deadzone
    // Offset measured at rear hardstop (approx -259 degrees)
    config.MagnetSensor.MagnetOffset =
        -0.4897 - TurretSubsystem.CANCODER_26T_TO_TURRET_GEAR_RATIO / 0.1;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;

    return config;
  }

  @Override
  public Command runHoodSysid() {
    return Commands.sequence(
        hoodSysid
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    hoodInputs.hoodPositionRotations.getDegrees()
                        > (HOOD_MAX_ANGLE.getDegrees() - 5)), // Stop before endstop
        hoodSysid
            .quasistatic(Direction.kReverse)
            .until(
                () ->
                    hoodInputs.hoodPositionRotations.getDegrees()
                        < (HOOD_MIN_ANGLE.getDegrees() + 5)),
        hoodSysid
            .dynamic(Direction.kForward)
            .until(
                () ->
                    hoodInputs.hoodPositionRotations.getDegrees()
                        > (HOOD_MAX_ANGLE.getDegrees() - 5)),
        hoodSysid
            .dynamic(Direction.kReverse)
            .until(
                () ->
                    hoodInputs.hoodPositionRotations.getDegrees()
                        < (HOOD_MIN_ANGLE.getDegrees() + 5)));
  }

  @Override
  public Command runFlywheelSysid() {
    return Commands.sequence(
        flywheelSysid.quasistatic(Direction.kForward),
        flywheelSysid.quasistatic(Direction.kReverse),
        flywheelSysid.dynamic(Direction.kForward),
        flywheelSysid.dynamic(Direction.kReverse));
  }
}
