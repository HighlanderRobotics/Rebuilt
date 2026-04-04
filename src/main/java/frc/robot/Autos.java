// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.ClimbTargets;
import frc.robot.utils.FieldUtils.TrenchPoses;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Autos {
  private final SwerveSubsystem swerve;
  private final ClimberSubsystem climber;
  private final AutoFactory factory;
  private static boolean autoFeed;
  private static boolean autoIntake;
  private static boolean autoScore;
  private static boolean autoPreClimb;
  private static boolean autoClimb;
  private static boolean autoFlow;
  private static boolean leftClimbAuto;

  @AutoLogOutput(key = "Superstructure/Auto Feed Request")
  public static Trigger autoFeedReq = new Trigger(() -> autoFeed).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Intake Request")
  public static Trigger autoIntakeReq =
      new Trigger(() -> autoIntake).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Score Request")
  public static Trigger autoScoreReq =
      new Trigger(() -> autoScore).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Pre Climb Request")
  public static Trigger autoPreClimbReq =
      new Trigger(() -> autoPreClimb).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Climb Request")
  public static Trigger autoClimbReq =
      new Trigger(() -> autoClimb).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Climb Request")
  public static Trigger autoFlowReq = new Trigger(() -> autoFlow).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Left Climb Request")
  public static Trigger autoLeftClimbReq =
      new Trigger(() -> leftClimbAuto).and(DriverStation::isAutonomous);

  public enum Action {
    FEED,
    INTAKE,
    SCORE,
    DELAYED_SCORE,
    FLOW,
    CLIMB_SCORE,
    OUTPOST,
    DEPOT,
    NOTHING,
    CLIMB_ONLY,
    SCORE_AT_END,
    OUTPOST_NO_SCORE;
  }

  public enum Path {
    // OUTPOST
    RTrenchtoOutpost("RTrenchtoOutpost", Action.OUTPOST),
    RPreTrenchReversedtoOutpost("RPreTrenchReversedtoOutpost", Action.OUTPOST),
    NoScoreRPreTrenchReversedtoOutpost("RPreTrenchReversedtoOutpost", Action.OUTPOST_NO_SCORE),

    PreOutposttoOutpost("PreOutposttoOutpost", Action.OUTPOST),

    HubtoOutpost("HubtoOutpost", Action.OUTPOST),
    // DEPOT
    LTrenchtoDepot("LTrenchtoDepot", Action.FLOW),
    LBumptoDepot("LBumptoDepot", Action.INTAKE),

    // FEED
    FeedLNeutraltoLPreTrench("LNeutraltoLPreTrench", Action.FEED),
    FeedRNeutraltoRPreTrench("RNeutraltoRPreTrench", Action.FEED),
    // INTAKE
    LNeutraltoLPreTrench("LNeutraltoLPreTrench", Action.INTAKE),

    EndWScoreLNeutraltoLPreTrench("LNeutraltoLPreTrench", Action.SCORE_AT_END),
    EndWScoreLCleanuptoLPreTrench("LCleanuptoLPreTrench", Action.SCORE_AT_END),

    RNeutraltoRPreTrenchReversed("RNeutraltoRPreTrenchReversed", Action.SCORE_AT_END),
    RDisrupttoRPreTrenchReversed("RDisrupttoRPreTrenchReversed", Action.SCORE_AT_END),
    LDisrupttoLPreTrench("LDisrupttoLPreTrench", Action.SCORE_AT_END),

    LPreTrenchtoLNeutral("LPreTrenchtoLNeutral", Action.INTAKE),
    LPreTrenchtoLCleanup("LPreTrenchtoLCleanup", Action.INTAKE),

    RPreTrenchtoRNeutral("RPreTrenchtoRNeutral", Action.INTAKE),
    RPreTrenchReversedtoRNeutral("RPreTrenchReversedtoRNeutral", Action.INTAKE),

    StartingRTrenchtoRNeutral("StartingRTrenchtoRNeutral", Action.INTAKE),
    StartingLTrenchtoLNeutral("StartingLTrenchtoLNeutral", Action.INTAKE),
    StartingRTrenchtoDisrupt("StartingRTrenchtoDisruptR", Action.INTAKE),
    StartingLTrenchtoDisrupt("StartingLTrenchtoDisruptL", Action.INTAKE),

    HubtoDepot("HubtoDepot", Action.DEPOT),

    PreDepottoDepot("PreDepottoDepot", Action.DEPOT),

    // SCORE
    DepottoLPreTrench("DepottoLPreTrench", Action.SCORE),
    OutposttoRPreTrench("OutposttoRPreTrench", Action.NOTHING),
    DepottoPreOutpost("DepottoPreOutpost", Action.DELAYED_SCORE),
    OutposttoPreDepot("OutposttoPreDepot", Action.NOTHING),
    OutposttoPreOutpost("OutposttoPreOutpost", Action.SCORE_AT_END),
    HubtoCenter("HubtoCenter", Action.SCORE),

    DepottoPreDepot("DepottoPreDepot", Action.SCORE_AT_END),
    // FLOW
    LPreTrenchtoDepot("LPreTrenchtoDepot", Action.FLOW),
    // CLIMB
    LPreTrenchtoLClimb("LPreTrenchtoLClimb", Action.CLIMB_SCORE),
    RPreTrenchtoRClimb("RPreTrenchtoRClimb", Action.CLIMB_SCORE),
    OutposttoRClimb("OutposttoRClimb", Action.CLIMB_SCORE),
    PreOutposttoRClimb("PreOutposttoRClimb", Action.CLIMB_ONLY), // uhh
    noScoreOutposttoRClimb("OutposttoRClimb", Action.CLIMB_ONLY),
    DepottoLClimb("DepottoLClimb", Action.CLIMB_SCORE),
    RBumptoOutpost("RBumptoOutpost", Action.OUTPOST),

    RUNtoTEST("RUNtoTEST", Action.NOTHING);

    private final String name;
    private final Action action;

    /**
     * @param name the name of the path in choreo. MUST match
     * @param action the action to perform during/at the end of the path
     */
    private Path(String name, Action action) {
      this.name = name;
      this.action = action;
    }

    public AutoTrajectory getTrajectory(AutoRoutine routine) {
      // AutoRoutine docs say that this "creates" a new trajectory, but the factory does check if
      // it's already present
      return routine.trajectory(name);
    }
  }

  // for the weird outpost paths idk
  public static final Pose2d BLUE_OUTPOST =
      new Pose2d(
          new Translation2d(0.44367337226867676, 0.443471223115921), Rotation2d.fromDegrees(90));
  public static final Pose2d RED_OUTPOST = ChoreoAllianceFlipUtil.flip(BLUE_OUTPOST);

  public static final Pose2d BLUE_DEPOT =
      new Pose2d(0.703999767303467, 5.975247383117676, Rotation2d.fromRadians(3.141592653589793));
  public static final Pose2d RED_DEPOT = ChoreoAllianceFlipUtil.flip(BLUE_DEPOT);

  public Autos(SwerveSubsystem swerve, ClimberSubsystem climber) {
    this.swerve = swerve;
    this.climber = climber;
    factory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetPose,
            swerve.choreoDriveController(),
            true,
            swerve,
            (traj, edge) -> {
              Logger.recordOutput(
                  "Choreo/Active Traj",
                  DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Blue)
                      ? traj.getPoses()
                      : traj.flipped().getPoses());
              Logger.recordOutput("Choreo/Active Traj Name", traj.name());
            });
  }

  public Command runPath(Path path, AutoRoutine routine) {
    Action action = path.action;
    switch (action) {
      case INTAKE:
        return intakePath(path, routine);
      case FEED:
        return feedFlowPath(path, routine);
      case SCORE:
        return scorePath(path, routine);
      case DELAYED_SCORE:
        return delayedScorePath(path, routine);
      case CLIMB_SCORE:
        return climbScorePath(path, routine);
      case FLOW:
        return flowScorePath(path, routine);
      case OUTPOST:
        return outpostPath(path, routine);
      case DEPOT:
        return depotPath(path, routine);
      case CLIMB_ONLY:
        return climbNoScorePath(path, routine);
      case SCORE_AT_END:
        return scoreAtEndPath(path, routine);
      case NOTHING:
        return emptyPath(path, routine);
      case OUTPOST_NO_SCORE:
        return outpostNoScorePath(path, routine);
      default: // this should never happen
        return Commands.none();
    }
  }

  public Command climbScorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        startFlowing(),
        startScoring(),
        // stopIntaking(),
        startPreClimb(),
        // Commands.parallel(
        path.getTrajectory(routine)
            .cmd()
            .until(
                // routine.observe(
                //     path.getTrajectory(routine)
                //         .atTime(
                //             path.getTrajectory(routine).getRawTrajectory().getTotalTime()
                //                 - (0.3)))),
                path.getTrajectory(routine).done()),
        stopScoring(),
        swerve.stop().repeatedly().until(() -> climber.atFullExtension()),
        Commands.parallel(
            swerve.alignToClimb(() -> getClimbAutoTarget()),
            Commands.waitUntil(
                    new Trigger(() -> swerve.isInAutoAimTolerance(getClimbAutoTarget().getPose()))
                        .debounce(0.2))
                .andThen(
                    // Commands.print("hooray!")
                    startClimb())));
  }

  public Command climbNoScorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        stopScoring(),
        stopIntaking(),
        startPreClimb(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        swerve.stop().repeatedly().until(() -> climber.atFullExtension()),
        Commands.parallel(
            swerve.alignToClimb(() -> getClimbAutoTarget()),
            Commands.waitUntil(
                    new Trigger(() -> swerve.isInAutoAimTolerance(getClimbAutoTarget().getPose()))
                        .debounce(0.2))
                .andThen(startClimb())));
  }

  public Command feedFlowPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        stopIntaking(),
        startFlowing(),
        startFeeding(),
        // startIntaking(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()));
  }

  public Command scorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        stopIntaking(),
        stopFeeding(),
        stopFlowing(),
        startScoring(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()));
  }

  public Command delayedScorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
            stopIntaking(),
            stopFeeding(),
            stopFlowing(),
            Commands.parallel(
                path.getTrajectory(routine).cmd(),
                Commands.sequence(
                    Commands.waitSeconds(1.2), Commands.print("Scoring!"), startScoring())))
        .until(path.getTrajectory(routine).done());
  }

  public Command scoreAtEndPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        startIntaking(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        stopIntaking(),
        startScoring(),
        swerve.stopForTime(() -> 5),
        stopScoring());
  }

  public Command emptyPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAllReqsFalse(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()));
  }

  public Command intakePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        stopScoring(),
        stopFlowing(),
        startIntaking(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()));
  }

  public Command flowScorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        stopIntaking(),
        startFlowing(),
        startScoring(),
        // setAutoIntakeReqTrue(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()));
  }

  public ClimbTargets getClimbAutoTarget() {
    return ClimbTargets.CLIMB_TARGETS_LIST.stream()
        .filter(target -> target.getLeftHanded() == leftClimbAuto)
        .filter(
            target ->
                target.isBlueAlliance()
                    == (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue))
        .findFirst()
        .get();
  }

  public Command outpostPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        stopScoring(),
        stopFeeding(),
        stopFlowing(),
        stopIntaking(),
        // holy chopped
        // this is here because i suspect the time based at end will cause it to stop early if it
        // hits something and messes up the time
        // though this may be fixed by having a better path?? idk
        path.getTrajectory(routine)
            .cmd()
            .until(
                path.getTrajectory(routine)
                    .atPose(
                        path.getTrajectory(routine)
                            .getFinalPose()
                            .orElse(
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                    ? BLUE_OUTPOST
                                    : RED_OUTPOST),
                        0.25,
                        Units.degreesToRadians(30))),
        startScoring(),
        // TODO tune tolerance
        swerve.stopForTime(() -> 4)); // TODO tune time
  }

  public Command outpostNoScorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        stopScoring(),
        stopFeeding(),
        stopFlowing(),
        stopIntaking(),
        // holy chopped
        // this is here because i suspect the time based at end will cause it to stop early if it
        // hits something and messes up the time
        // though this may be fixed by having a better path?? idk
        path.getTrajectory(routine)
            .cmd()
            .until(
                path.getTrajectory(routine)
                    .atPose(
                        path.getTrajectory(routine)
                            .getFinalPose()
                            .orElse(
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                    ? BLUE_OUTPOST
                                    : RED_OUTPOST),
                        0.25,
                        Units.degreesToRadians(30))),
        // TODO tune tolerance
        swerve.stopForTime(() -> 4)); // TODO tune time
  }

  public Command depotPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        stopScoring(),
        stopFeeding(),
        stopFlowing(),
        startIntaking(),
        // holy chopped
        // this is here because i suspect the time based at end will cause it to stop early if it
        // hits something and messes up the time
        // though this may be fixed by having a better path?? idk
        path.getTrajectory(routine)
            .cmd()
            .until(
                path.getTrajectory(routine)
                    .atPose(
                        path.getTrajectory(routine)
                            .getFinalPose()
                            .orElse(
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                    ? BLUE_DEPOT
                                    : RED_DEPOT),
                        0.25,
                        Units.degreesToRadians(30))),
        // TODO tune tolerance
        swerve.stopForTime(() -> 1)); // TODO tune time
  }

  public void lockHoodUnderTrench(AutoRoutine routine, double toleranceMeters) {
    routine
        .observe(
            () ->
            // swerve.getPose().getTranslation().minus(trench.getTranslation()).getNorm()
            // swerve.getPose().minus(trench).getTranslation().getNorm() < tolerance)
            {
              for (TrenchPoses t : FieldUtils.TrenchPoses.values()) {
                if (swerve.getPose().minus(t.getPose()).getTranslation().getNorm()
                    < toleranceMeters) return true;
              }
              return false;
            })
        .whileTrue(Commands.run(() -> stopScoring()));
  }

  public Command shootPreload() {
    return Commands.sequence(startScoring(), swerve.stop().repeatedly().withTimeout(3));
  }

  public Command startIntaking() {
    return Commands.runOnce(() -> autoIntake = true);
  }

  public Command stopIntaking() {
    return Commands.runOnce(() -> autoIntake = false);
  }

  public Command startScoring() {
    return Commands.runOnce(() -> autoScore = true);
  }

  public Command stopScoring() {
    return Commands.runOnce(() -> autoScore = false);
  }

  public Command startFeeding() {
    return Commands.runOnce(() -> autoFeed = true);
  }

  public Command stopFeeding() {
    return Commands.runOnce(() -> autoFeed = false);
  }

  public Command startPreClimb() {
    return Commands.runOnce(() -> autoPreClimb = true);
  }

  public Command stopPreClimb() {
    return Commands.runOnce(() -> autoPreClimb = false);
  }

  public Command startFlowing() {
    return Commands.runOnce(() -> autoFlow = true);
  }

  public Command stopFlowing() {
    return Commands.runOnce(() -> autoFlow = false);
  }

  public Command startClimb() {
    return Commands.runOnce(() -> autoClimb = true);
  }

  public Command stopClimb() {
    return Commands.runOnce(() -> autoClimb = false);
  }

  public Command setAllReqsFalse() {
    return Commands.sequence(
        stopIntaking(), stopScoring(), stopFeeding(), stopPreClimb(), stopFlowing(), stopClimb());
  }

  public void setAllReqsFalsenotcmd() {
    autoIntake = false;
    autoScore = false;
    autoFeed = false;
    autoPreClimb = false;
    autoFlow = false;
    autoClimb = false;
  }

  public Command setLeftClimb() {
    return Commands.runOnce(() -> leftClimbAuto = true);
  }

  public Command setRightClimb() {
    return Commands.runOnce(() -> leftClimbAuto = false);
  }

  public Command createAuto(
      String name, Path[] paths, Command setClimbSideCmd, Command... startingCommands) {
    final AutoRoutine routine = factory.newRoutine(name);
    lockHoodUnderTrench(routine, 1);

    Command autoCommand =
        paths[0]
            .getTrajectory(routine)
            .resetOdometry()
            .alongWith(setClimbSideCmd)
            .andThen(startingCommands);
    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }
    routine.active().onTrue(autoCommand);
    return routine.cmd();
  }

  public Command getDepotScoreClimbAuto() {
    return createAuto(
        "Depot Score Climb Auto",
        new Path[] {
          Path.LTrenchtoDepot,
          Path.DepottoLPreTrench,
          Path.LPreTrenchtoLNeutral,
          Path.LNeutraltoLPreTrench,
          Path.LPreTrenchtoLClimb
        },
        setLeftClimb());
  }

  public Command getDoubleDipRightTrench() {
    return createAuto(
        "Double dip right trench auto",
        new Path[] {
          Path.StartingRTrenchtoRNeutral,
          Path.RNeutraltoRPreTrenchReversed,
          Path.RPreTrenchReversedtoRNeutral,
          Path.RNeutraltoRPreTrenchReversed,
        },
        setRightClimb());
  }

  public Command getDisruptOutpostRightTrench() {
    return createAuto(
        "Disrupt Outpost Right Trench Auto",
        new Path[] {
          Path.StartingRTrenchtoDisrupt,
          Path.RDisrupttoRPreTrenchReversed,
          Path.RPreTrenchReversedtoOutpost,
        },
        setRightClimb());
  }

  public Command getDisruptDepotLeftTrench() {
    return createAuto(
        "Disrupt Depot Left Trench Auto",
        new Path[] {
          Path.StartingLTrenchtoDisrupt, Path.LDisrupttoLPreTrench, Path.LPreTrenchtoDepot
        },
        setLeftClimb());
  }

  public Command getOutpostScoreClimbAuto() {

    return createAuto(
        "Outpost Score Climb Auto",
        new Path[] {
          Path.RTrenchtoOutpost,
          Path.OutposttoRPreTrench,
          Path.RPreTrenchtoRNeutral,
          Path.RNeutraltoRPreTrenchReversed,
          Path.RPreTrenchtoRClimb
        },
        setRightClimb());
  }

  public Command getDepotFeedClimbAuto() {
    return createAuto(
        "Depot Feed Climb Auto",
        new Path[] {
          Path.LTrenchtoDepot,
          Path.DepottoLPreTrench,
          Path.LPreTrenchtoLNeutral,
          Path.FeedLNeutraltoLPreTrench,
          Path.LPreTrenchtoLClimb
        },
        setLeftClimb());
  }

  public Command getOutpostFeedClimbAuto() {

    return createAuto(
        "Outpost Feed Climb Auto",
        new Path[] {
          Path.RTrenchtoOutpost,
          Path.OutposttoRPreTrench,
          Path.RPreTrenchtoRNeutral,
          Path.FeedRNeutraltoRPreTrench,
          Path.RPreTrenchtoRClimb
        },
        setRightClimb());
  }

  // awful names.. mb
  public Command getFillDepotScoreClimbAuto() {
    return createAuto(
        "Fill Depot Score Climb Auto",
        new Path[] {
          Path.StartingLTrenchtoLNeutral,
          Path.FeedLNeutraltoLPreTrench,
          Path.LPreTrenchtoDepot,
          Path.DepottoLClimb
        },
        setLeftClimb());
  }

  public Command getFillOutpostScoreClimbAuto() {

    return createAuto(
        "Fill Outpost Score Climb Auto",
        new Path[] {
          Path.StartingRTrenchtoRNeutral,
          Path.FeedRNeutraltoRPreTrench,
          Path.RPreTrenchReversedtoOutpost,
          Path.OutposttoRClimb
        },
        setRightClimb());
  }

  public Command getRightBumpOutpostCenterAuto() {

    return createAuto(
        "Right Bump Outpost Center Auto",
        new Path[] {
          Path.RBumptoOutpost,
          Path.OutposttoRPreTrench,
          Path.RPreTrenchtoRNeutral,
          Path.RNeutraltoRPreTrenchReversed
        },
        setRightClimb());
  }

  public Command getDepotClimbAuto() {

    return createAuto(
        "Depot Climb Auto", new Path[] {Path.LTrenchtoDepot, Path.DepottoLClimb}, setLeftClimb());
  }

  public Command getOutpostClimbAuto() {

    return createAuto(
        "Outpost Climb Auto",
        new Path[] {Path.RTrenchtoOutpost, Path.OutposttoPreOutpost, Path.PreOutposttoRClimb},
        setRightClimb());
  }

  public Command getDepotOutpostClimbAuto() {
    return createAuto(
        "Depot Outpost Climb Auto",
        new Path[] {
          Path.LTrenchtoDepot,
          Path.DepottoPreOutpost,
          Path.PreOutposttoOutpost,
          Path.OutposttoRClimb
        },
        setRightClimb());
  }

  public Command getLeftBumpDepotOutpostClimbAuto() {
    return createAuto(
        "Left Bump Outpost Climb Auto",
        new Path[] {
          Path.LBumptoDepot, Path.DepottoPreOutpost, Path.PreOutposttoOutpost, Path.OutposttoRClimb
        },
        setRightClimb(),
        shootPreload());
  }

  public Command getHubDepotOutpostAuto() {
    return createAuto(
        "Hub Depot Outpost Auto",
        new Path[] {Path.HubtoDepot, Path.DepottoPreOutpost, Path.PreOutposttoOutpost},
        Commands.none());
  }

  public Command getHubOutpostDepotAuto() {
    return createAuto(
        "Hub Outpost Depot Auto",
        new Path[] {
          Path.HubtoOutpost, Path.OutposttoPreDepot, Path.PreDepottoDepot, Path.DepottoPreDepot
        },
        Commands.none());
  }

  // this is so cursed and im not proud of it
  public Command getRightBumpOutpostClimbAuto() {
    return createAuto(
        "Right Bump Outpost Climb Auto",
        new Path[] {Path.RBumptoOutpost, Path.noScoreOutposttoRClimb},
        setRightClimb());
  }

  public Command getCenterScoreAuto() {
    // no climb so don't need to set climb target
    return createAuto("Center Score Auto", new Path[] {Path.HubtoCenter}, Commands.none());
  }

  public Command getRightNeutralOutpostScore() {
    return createAuto(
        "Right Neutral Outpost Score",
        new Path[] {
          Path.StartingRTrenchtoRNeutral,
          Path.RNeutraltoRPreTrenchReversed,
          Path.NoScoreRPreTrenchReversedtoOutpost,
          Path.OutposttoPreOutpost
        },
        setRightClimb());
  }

  // public Command getLeftNeutralOutpostScore() {
  //   return createAuto(
  //       "Left Neutral Outpost Score",
  //       new Path[] {
  //         Path.StartingLTrenchtoLNeutral, Path.LNeutraltoLPreTrench, Path.LPreTrenchtoDepot
  //       },
  //       setLeftClimb());
  // }

  public Command getLeftNeutralScoreTwice() {
    return createAuto(
        "Left Neutral Score Twice",
        new Path[] {
          Path.StartingLTrenchtoLNeutral,
          Path.EndWScoreLNeutraltoLPreTrench,
          Path.LPreTrenchtoLCleanup,
          Path.EndWScoreLCleanuptoLPreTrench
        },
        setLeftClimb());
  }

  public Command getTestAuto() {
    final AutoRoutine routine = factory.newRoutine("test auto");
    Path[] paths = {Path.RUNtoTEST, Path.RUNtoTEST, Path.RUNtoTEST, Path.RUNtoTEST};
    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry();

    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }

    routine.active().whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command waitUntilEmpty() {
    return Commands.waitSeconds(3.0);
  }

  public Command getJustScoreAuto() {
    return startScoring().andThen(Commands.waitSeconds(5)).andThen(stopScoring());
  }
}
