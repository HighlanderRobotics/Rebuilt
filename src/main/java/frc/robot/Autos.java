// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
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
    FLOW,
    CLIMB_SCORE,
    OUTPOST,
    NOTHING,
    OUTPOST_SCORE,
    CLIMB_ONLY,
    INTAKE_SCORE;
  }

  public enum Obstacle {
    TRENCH,
    BUMP;
  }

  /* NEW NAMING  (cooked)

  #1
  I for intake
  F for feed
  S for score
  (indicate action for that path/pose)

  P for park
  (starting poses)

  #2
  L for left
  R for right
  (indicate starting on left or right side)

  #3
  T for trench
  B for bump
  (each routine has a varition) (only for crossing paths ig)

  climb no climb variations

  R is a middle point facing towards the neutral zone
  M is middle poitns facing our alliance
    */

  public enum Path {
    // OUTPOST
    PRtoO("PR", "O", Action.OUTPOST),
    MRtoO("MR", "O", Action.OUTPOST),
    StoO("S", "O", Action.OUTPOST),
    // DEPOT
    PLtoD("PL", "D", Action.INTAKE),
    // FEED
    FLtoFLM("FL", "FLM", Action.FEED),
    FRtoFRM("FR", "FRM", Action.FEED),
    FLMtoML("FLM", "ML", Action.FEED),
    FRMtoMR("FRM", "MR", Action.FEED),
    // INTAKE
    ILtoILM("FL", "FLM", Action.INTAKE),
    IRtoIRM("FR", "FRM", Action.INTAKE),
    ILMtoML("FLM", "ML", Action.INTAKE),
    IRMtoMR("FRM", "MR", Action.INTAKE),
    RLtoIL("RL", "FL", Action.INTAKE),
    RRtoIR("RR", "FR", Action.INTAKE),
    PRtoIR("PR", "FR", Action.INTAKE),
    PLtoIL("PL", "FL", Action.INTAKE),
    // SCORE
    DtoRL("D", "RL", Action.SCORE),
    OtoRR("O", "RR", Action.NOTHING),
    DtoS("D", "S", Action.SCORE),
    OtoS("O", "S", Action.SCORE),
    PMtoM("PM", "M", Action.SCORE),
    // FLOW
    MLtoD("ML", "D", Action.FLOW),
    // CLIMB
    MLtoCL("ML", "CL", Action.CLIMB_SCORE),
    MRtoCR("MR", "CR", Action.CLIMB_SCORE),
    OtoCR("O", "CR", Action.CLIMB_SCORE),
    noScoreOtoCR("O", "CR", Action.CLIMB_ONLY),
    DtoCL("D", "CL", Action.CLIMB_SCORE),
    RBtoO("RB", "O", Action.OUTPOST_SCORE),

    FRMtoMRScore("FRM", "MR", Action.INTAKE_SCORE),

    RUNtoTEST("RUN", "TEST", Action.NOTHING),

    BtoD("B", "D", Action.INTAKE);

    private final String start;
    private final String end;
    private final Action action;

    private Path(String start, String end, Action action) {
      this.start = start;
      this.end = end;
      this.action = action;
    }

    public AutoTrajectory getTrajectory(AutoRoutine routine) {
      // AutoRoutine docs say that this "creates" a new trajectory, but the factory does check if
      // it's already present
      return routine.trajectory(start + "to" + end);
    }
  }

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

  public Command leaveAuto() {
    final AutoRoutine routine = factory.newRoutine("Leave Auto");
    Path[] paths = {};

    Command autoCommand = Commands.none();

    for (Path path : paths) {
      autoCommand =
          autoCommand.andThen(
              Commands.print("Running path: " + path.toString()).andThen(runPath(path, routine)));
    }

    routine.active().whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command runPath(Path path, AutoRoutine routine) {
    Action action = path.action;
    switch (action) {
      case INTAKE:
        return intakePath(path, routine);
      case FEED:
        return feedPath(path, routine);
      case SCORE:
        return scorePath(path, routine);
      case CLIMB_SCORE:
        return climbScorePath(path, routine);
      case FLOW:
        return flowPath(path, routine);
      case OUTPOST:
        return outpostPath(path, routine);
      case OUTPOST_SCORE:
        return outpostScorePath(path, routine);
      case CLIMB_ONLY:
        return climbNoScorePath(path, routine);
      case INTAKE_SCORE:
        return intakeScorePath(path, routine);
      case NOTHING:
        return emptyPath(path, routine);
      default: // this should never happen
        return Commands.none();
    }
  }

  public Command climbScorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoScoreReqFalse(),
        setAutoIntakeReqFalse(),

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
        Commands.parallel(swerve.stop(), setAutoScoreReqTrue()).repeatedly().withTimeout(2.5),
        setAutoScoreReqFalse(),
        setAutoPreClimbReqTrue(),
        swerve.stop().until(() -> climber.atFullExtension()),
        Commands.parallel(
            swerve.alignToClimb(() -> getClimbAutoTarget()),
            Commands.waitUntil(
                    new Trigger(() -> swerve.isInAutoAimTolerance(getClimbAutoTarget().getPose()))
                        .debounce(0.2))
                .andThen(
                    // Commands.print("hooray!")
                    setAutoClimbReqTrue())));
  }

  public Command climbNoScorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoScoreReqFalse(),
        setAutoIntakeReqFalse(),
        setAutoPreClimbReqTrue(),
        // Commands.parallel(
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        swerve.stop().until(() -> climber.atFullExtension()),
        Commands.parallel(
            swerve.alignToClimb(() -> getClimbAutoTarget()),
            Commands.waitUntil(
                    new Trigger(() -> swerve.isInAutoAimTolerance(getClimbAutoTarget().getPose()))
                        .debounce(0.2))
                .andThen(setAutoClimbReqTrue())));
  }

  public Command feedPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoScoreReqFalse(),
        setAutoFeedReqTrue(),
        setAutoIntakeReqTrue(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        setAutoFeedReqFalse());
  }

  public Command scorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoIntakeReqFalse(),
        // setAutoScoreReqTrue(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        // setAutoScoreReqTrue()
        // ,
        // setAutoScoreReqFalse()
        setAutoScoreReqTrue(),
        swerve.stop().repeatedly().withTimeout(3));
  }

  public Command emptyPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()));
  }

  public Command intakePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoScoreReqFalse(),
        setAutoFlowReqFalse(),
        setAutoIntakeReqTrue(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        setAutoIntakeReqFalse());
  }

  public Command intakeScorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoScoreReqFalse(),
        setAutoFlowReqFalse(),
        setAutoIntakeReqTrue(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        setAutoIntakeReqFalse(),
        setAutoScoreReqTrue(),
        swerve.stop().repeatedly().withTimeout(4),
        setAutoScoreReqFalse());
  }

  public Command flowPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoScoreReqTrue(),
        setAutoFlowReqTrue(),
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
        setAutoScoreReqFalse(),
        setAutoFlowReqFalse(),
        setAutoIntakeReqFalse(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        swerve.stop().repeatedly().withTimeout(2)
        // Commands.waitSeconds(1)
        );
  }

  public Command outpostScorePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoScoreReqFalse(),
        setAutoFlowReqFalse(),
        setAutoIntakeReqFalse(),
        // spin up before we get there
        // Commands.parallel(
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        // Commands.waitUntil(path.getTrajectory(routine).atTimeBeforeEnd(0.2))
        // .andThen(
        setAutoScoreReqTrue()
        // ))
        ,
        swerve.stop().repeatedly().withTimeout(4),
        setAutoScoreReqFalse()
        // Commands.waitSeconds(1)
        );
  }

  public void lockHoodUnderTrench(AutoRoutine routine, Pose2d trench, double tolerance) {
    routine
        .observe(
            () ->
                // swerve.getPose().getTranslation().minus(trench.getTranslation()).getNorm()
                swerve.getPose().minus(trench).getTranslation().getNorm() < tolerance)
        .whileTrue(Commands.run(() -> setAutoScoreReqFalse()));
  }

  public Command shootPreload() {
    return Commands.sequence(setAutoScoreReqTrue(), swerve.stop().repeatedly().withTimeout(3));
  }

  public Command setAutoIntakeReqTrue() {
    return Commands.runOnce(() -> autoIntake = true);
  }

  public Command setAutoIntakeReqFalse() {
    return Commands.runOnce(() -> autoIntake = false);
  }

  public Command setAutoScoreReqTrue() {
    return Commands.runOnce(() -> autoScore = true);
  }

  public Command setAutoScoreReqFalse() {
    return Commands.runOnce(() -> autoScore = false);
  }

  public Command setAutoFeedReqTrue() {
    return Commands.runOnce(() -> autoFeed = true);
  }

  public Command setAutoFeedReqFalse() {
    return Commands.runOnce(() -> autoFeed = false);
  }

  public Command setAutoPreClimbReqTrue() {
    return Commands.runOnce(() -> autoPreClimb = true);
  }

  public Command setAutoPreClimbReqFalse() {
    return Commands.runOnce(() -> autoPreClimb = false);
  }

  public Command setAutoFlowReqTrue() {
    return Commands.runOnce(() -> autoFlow = true);
  }

  public Command setAutoFlowReqFalse() {
    return Commands.runOnce(() -> autoFlow = false);
  }

  public Command setAutoClimbReqTrue() {
    return Commands.runOnce(() -> autoClimb = true);
  }

  public Command setAutoClimbReqFalse() {
    return Commands.runOnce(() -> autoClimb = false);
  }

  public Command setAllReqsFalse() {
    return Commands.sequence(
        setAutoIntakeReqFalse(),
        setAutoScoreReqFalse(),
        setAutoFeedReqFalse(),
        setAutoPreClimbReqFalse(),
        setAutoFlowReqFalse(),
        setAutoClimbReqFalse());
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
    lockHoodUnderTrench(
        routine,
        TrenchPoses.getClosestTrenchPose(swerve.getPose()),
        1); // TODO the lock under trench feels suspect

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
        new Path[] {Path.PLtoD, Path.DtoRL, Path.RLtoIL, Path.ILMtoML, Path.MLtoCL},
        setLeftClimb());
  }

  public Command getOutpostScoreClimbAuto() {

    return createAuto(
        "Outpost Score Climb Auto",
        new Path[] {Path.PRtoO, Path.OtoRR, Path.RRtoIR, Path.IRtoIRM, Path.IRMtoMR, Path.MRtoCR},
        setRightClimb());
  }

  public Command getDepotFeedClimbAuto() {
    return createAuto(
        "Depot Feed Climb Auto",
        new Path[] {Path.PLtoD, Path.DtoRL, Path.RLtoIL, Path.FLtoFLM, Path.FLMtoML, Path.MLtoCL},
        setLeftClimb());
  }

  public Command getOutpostFeedClimbAuto() {

    return createAuto(
        "Outpost Feed Climb Auto",
        new Path[] {Path.PRtoO, Path.OtoRR, Path.RRtoIR, Path.FRtoFRM, Path.FRMtoMR, Path.MRtoCR},
        setRightClimb());
  }

  // awful names.. mb
  public Command getFillDepotScoreClimbAuto() {

    return createAuto(
        "Fill Depot Score Climb Auto",
        new Path[] {Path.PLtoIL, Path.FLtoFLM, Path.FLMtoML, Path.MLtoD, Path.DtoCL},
        setLeftClimb());
  }

  public Command getFillOutpostScoreClimbAuto() {

    return createAuto(
        "Fill Outpost Score Climb Auto",
        new Path[] {Path.PRtoIR, Path.FRtoFRM, Path.FRMtoMR, Path.MRtoO, Path.OtoCR},
        setRightClimb());
  }

  public Command getRightBumpOutpostCenterAuto() {

    return createAuto(
        "Right Bump Outpost Center Auto",
        new Path[] {Path.RBtoO, Path.OtoRR, Path.RRtoIR, Path.IRtoIRM, Path.FRMtoMRScore},
        setRightClimb());
  }

  public Command getDepotClimbAuto() {

    return createAuto("Depot Climb Auto", new Path[] {Path.PLtoD, Path.DtoCL}, setLeftClimb());
  }

  public Command getOutpostClimbAuto() {

    return createAuto(
        "Outpost Climb Auto", new Path[] {Path.PRtoO, Path.OtoS, Path.OtoCR}, setRightClimb());
  }

  public Command getDepotOutpostClimbAuto() {
    return createAuto(
        "Depot Outpost Climb Auto",
        new Path[] {Path.PLtoD, Path.DtoS, Path.StoO, Path.OtoCR},
        setRightClimb());
  }

  public Command getLeftBumpDepotOutpostClimbAuto() {
    return createAuto(
        "Left Bump Outpost Climb Auto",
        new Path[] {Path.BtoD, Path.DtoS, Path.StoO, Path.OtoCR},
        setRightClimb(),
        shootPreload());
  }

  // this is so cursed and im not proud of it
  public Command getRightBumpOutpostClimbAuto() {
    return createAuto(
        "Right Bump Outpost Climb Auto",
        new Path[] {Path.RBtoO, Path.noScoreOtoCR},
        setRightClimb());
  }

  public Command getCenterScoreAuto() {
    // no climb so don't need to set climb target
    return createAuto("Center Score Auto", new Path[] {Path.PMtoM}, Commands.none());
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
    return setAutoScoreReqTrue().andThen(Commands.waitSeconds(5)).andThen(setAutoScoreReqFalse());
  }
}
