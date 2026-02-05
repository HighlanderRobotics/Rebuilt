// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Autos {
  private final AutoFactory factory;
  // Declare triggers
  // mehhhhhhh
  private static boolean autoFeed;
  private static boolean autoIntake;
  private static boolean autoScore;
  private static boolean autoClimb;

  // private static boolean autoIntakeAlgae;

  @AutoLogOutput(key = "Superstructure/Auto Feed Request")
  public static Trigger autoFeedReq = new Trigger(() -> autoFeed).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Intake Request")
  public static Trigger autoIntakeReq =
      new Trigger(() -> autoIntake).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Score Request")
  public static Trigger autoScoreReq =
      new Trigger(() -> autoScore).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Climb Request")
  public static Trigger autoClimbReq =
      new Trigger(() -> autoClimb).and(DriverStation::isAutonomous);

  public enum Action {
    FEED,
    INTAKE,
    SCORE,
    FLOW,
    CLIMB;
  }

  public enum Obstacle {
    TRENCH,
    BUMP;
  }

  public enum Path {
    DtoFL("DT", "FL", Action.FEED), //
    FLMtoCL("FLM", "CL", Action.CLIMB), //
    FLMtoSL("FLM", "SLT", Action.SCORE), //
    FLtoFLM("FL", "FLM", Action.FEED), //
    // FLtoSL("FL", "SL", Action.SCORE), //not even used
    FRMtoCR("FRM", "CR", Action.CLIMB), // todo make the bumper vs trench
    FRMtoSR("FRM", "SR", Action.SCORE), // todo make the bumper vs trench
    FRtoFRM("FR", "FRM", Action.FEED), //
    // FRtoSR("FR", "SR", Action.SCORE), //not used so whatever
    OtoFR("OT", "FR", Action.FEED), //
    SLtoCL("SL", "CL", Action.CLIMB), //
    SLtoCM("SL", "CM", Action.CLIMB), //
    SLtoFL("SLT", "FL", Action.FEED), // todo make the bumper vs trench
    SRtoCM("SR", "CM", Action.CLIMB), //
    SRtoCR("SRT", "CR", Action.CLIMB), // this name is incorrect
    SRtoFR("SR", "FR", Action.FEED), // wait why is this needed
    PRtoO("PRT", "O", Action.FLOW), //
    PLtoD("PLT", "D", Action.FLOW), // here make intake and SCORE
    DtoIL("DT", "FL", Action.FLOW), //
    ILMtoSL("FLM", "SLT", Action.SCORE),
    ILtoILM("FL", "FLM", Action.INTAKE),
    // ILtoSL("FL", "SL", Action.SCORE), //not even used
    IRMtoSR("FRM", "SR", Action.SCORE), // todo make the bumper vs trench
    IRtoIRM("FR", "FRM", Action.INTAKE), // todo make the bumper vs trench
    // IRtoSR("FR", "SR", Action.SCORE), //not even used
    OtoIR("OT", "FR", Action.FLOW),
    // SLtoIL("SL", "FL", Action.INTAKE), //not even used
    //  SRtoIR("SR", "FR", Action.INTAKE); //not even used
    // better naming for DLt etc
    ILMtoML("FLM", "ML", Action.INTAKE),
    MLtoSL("ML", "SL", Action.SCORE),
    IRMtoMR("FRM", "MR", Action.INTAKE),
    MRtoSR("MR", "SR", Action.SCORE),
    // feeding ones
    // TODO organgize
    FLMtoML("FLM", "ML", Action.FEED),
    FRMtoMR("FRM", "MR", Action.FEED),
    MLtoCL("ML", "CL", Action.CLIMB),
    MRtoCR("MR", "CR", Action.CLIMB),
    DtoR("DT", "R", Action.SCORE),
    RtoFL("R", "FL", Action.FEED);



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

  public Autos(SwerveSubsystem swerve) {
    factory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetPose,
            swerve.choreoDriveController(),
            true,
            swerve,
            (traj, edge) -> {
              //   if (Robot.ROBOT_MODE != RobotMode.REAL)
              Logger.recordOutput(
                  "Choreo/Active Traj",
                  DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Blue)
                      ? traj.getPoses()
                      : traj.flipped().getPoses());
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
      case CLIMB:
        return climbPath(path, routine);
      case FLOW:
        return flowPath(path, routine);
      default: // this should never happen
        return Commands.none();
    }
  }

  // TODO aligning to climb pos correctly
  public Command climbPath(Path path, AutoRoutine routine) {
    // path align and climb

    return Commands.sequence(
        setAutoScoreReqTrue(),
        setAutoIntakeReqFalse(),
        path.getTrajectory(routine)
            .cmd()
            .until(
                routine.observe(
                    path.getTrajectory(routine)
                        .atTime(
                            path.getTrajectory(routine).getRawTrajectory().getTotalTime()
                                - (0.3)))),
        setAutoScoreReqFalse(),
        setAutoClimbReqTrue());
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
    // path align and score
    return Commands.sequence(
        setAutoIntakeReqFalse(),
        setAutoScoreReqTrue(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        // // .getRawTrajectory().getTotalTime()
        // //  - (0.3)))),
        setAutoScoreReqFalse());

    // Commands.waitUntil(inScoringArea()).andThen(setAutoScoreReqTrue()).alongWith(
    // Commands.sequence(path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
    //   setAutoScoreReqFalse()));
  }

  public Command intakePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoScoreReqFalse(),
        setAutoIntakeReqTrue(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        setAutoIntakeReqFalse());
  }

  public Command flowPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoScoreReqTrue(),
        setAutoIntakeReqTrue(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()));
  }

  public Command shootPreload() {
    return Commands.sequence(setAutoScoreReqTrue(), waitUntilEmpty(), setAutoScoreReqFalse());
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

  public Command setAutoClimbReqTrue() {
    return Commands.runOnce(() -> autoClimb = true);
  }

  public Command setAutoClimbReqFalse() {
    return Commands.runOnce(() -> autoClimb = false);
  }

  public Command getDepotScoreClimbAuto() {
    final AutoRoutine routine = factory.newRoutine("Depot Score Climb Auto");
    Path[] paths = {
      Path.PLtoD, Path.DtoIL, Path.ILtoILM, Path.ILMtoML, Path.MLtoCL
    }; // , Path.SLtoCL};
    Command autoCommand =
        paths[0].getTrajectory(routine).resetOdometry(); // .andThen(shootPreload());

    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }

    routine.active().onTrue(autoCommand);

    return routine.cmd();
  }

  public Command getOutpostScoreClimbAuto() {
    final AutoRoutine routine = factory.newRoutine("Outpost Score Climb Auto");
    Path[] paths = {Path.PRtoO, Path.OtoIR, Path.IRtoIRM, Path.IRMtoMR, Path.MRtoSR, Path.SRtoCR};
    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry();

    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }

    routine.active().onTrue(autoCommand);

    return routine.cmd();
  }

  public Command getDepotFeedClimbAuto() {
    final AutoRoutine routine = factory.newRoutine("Depot Feed Climb Auto");
    Path[] paths = {Path.PLtoD, Path.DtoR, Path.RtoFL, Path.FLtoFLM, Path.FLMtoML, Path.MLtoCL};
    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry();

    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }

    routine.active().whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command getOutpostFeedClimbAuto() {
    final AutoRoutine routine = factory.newRoutine("Outpost Feed Climb Auto");
    Path[] paths = {Path.PRtoO, Path.OtoFR, Path.FRtoFRM, Path.FRMtoMR, Path.MRtoCR};
    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry();

    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }

    routine.active().whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command getTestAuto() {
    final AutoRoutine routine = factory.newRoutine("Outpost Feed Climb Auto");
    Path[] paths = {Path.PLtoD};
    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry();

    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }

    routine.active().whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command waitUntilEmpty() {
    // TODO wait till robot empty / done scoring
    // return null;
    return Commands.waitSeconds(3.0);
  }

  /* NEW NAMING system (cooked)

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

  i will have arrays for the poses then string them together
  also indcate with argument like T vs B
    */

  // // Path[] paths = {Path.PRtoO, Path.OtoFR, Path.FRtoFRM, Path.FRMtoCR};
  // String[] outpostFeed = {"PR", "O", "FR", "FRM", "CR", "ER"};

  // // Path[] paths = {Path.PLtoD, Path.DtoFL, Path.FLtoFLM, Path.FLMtoCL};
  // String[] depotFeed = {"PL", "D", "FL", "FLM", "CL", "EL"};

  // // what does it do instead of climb?

  // // Path[] paths = {Path.PRtoO, Path.OtoIR, Path.IRtoIRM, Path.IRMtoSR, Path.SRtoCR};
  // String[] outpostScore = {"PR", "O", "IR", "IRM", "SR", "CR", "ER"};

  // // Path[] paths = {Path.PLtoD, Path.DtoIL, Path.ILtoILM, Path.ILMtoSL, Path.SLtoCL};
  // String[] depotScore = {"PL", "D", "IL", "ILM", "SL", "CL", "EL"};

  // String[] leftDisruptFeed = {"PL", "DLO", "DLT", "CL", "EL"};

  // String[] rightDisruptFeed = {"PR", "DRO", "DRT", "CR", "ER"};

  // // make these able to go back and fourth
  // // idea EL and ER some random ahh end poses

  // public Command makeAutos(
  //     String name, String[] positions, Obstacle trenchOrBump, boolean climb, boolean preScore) {
  //   final AutoRoutine routine = factory.newRoutine(name);

  //   Path[] paths = {};
  //   for (String p : positions) {
  //     // paths// (add paths and also adjust for climbing and bump vs trench )
  //   }
  //   Command autoCommand =
  //       paths[0]
  //           .getTrajectory(routine)
  //           .resetOdometry()
  //           .andThen(preScore ? shootPreload() : Commands.none());

  //   for (Path p : paths) {
  //     autoCommand = autoCommand.andThen(runPath(p, routine));
  //   }

  //   routine.active().whileTrue(autoCommand);

  //   return routine.cmd();
  // }
}
