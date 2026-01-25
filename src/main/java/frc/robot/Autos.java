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
import frc.robot.Robot.RobotMode;
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
    CLIMB;
  }

  public enum Path {
    // R for right
    // L for left
    // M for middle
    // P for park (starting pose but often used for scoring pose ig)
    // D for depot
    // O for outpost
    // C for climb
    // S  was going to  be for scoreing pos but i think we will just score
    // F for feeding poses
    // I for intake???

    // may have to rethink naming to some extent and add more poses

    DtoFL("D", "FL", Action.FEED),
    FLMtoPL("FLM", "PL", Action.SCORE),
    FLtoFLM("FL", "FLM", Action.FEED),
    FLtoPL("FL", "PL", Action.SCORE),
    FRMtoPR("FRM", "PR", Action.SCORE),
    FRtoFRM("FR", "FRM", Action.FEED),
    FRtoPR("FR", "PR", Action.SCORE),
    OtoFR("O", "FR", Action.FEED),
    PLtoCL("PL", "CL", Action.CLIMB),
    PLtoCM("PL", "CM", Action.CLIMB),
    PLtoD("PL", "D", Action.INTAKE),
    PLtoFL("PL", "FL", Action.FEED),
    PRtoCM("PR", "CM", Action.CLIMB),
    PRtoCR("PR", "CR", Action.CLIMB),
    PRtoFR("PR", "FR", Action.FEED),
    PRtoO("PR", "O", Action.INTAKE),
    // idk seperate intake and feed so action is included makes it easier for me but they use the
    // same
    // trajectories so i dont have to make new paths
    DtoIL("D", "FL", Action.INTAKE),
    ILMtoPL("FLM", "PL", Action.SCORE),
    ILtoILM("FL", "FLM", Action.INTAKE),
    ILtoPL("FL", "PL", Action.SCORE),
    IRMtoPR("FRM", "PR", Action.SCORE),
    IRtoIRM("FR", "FRM", Action.INTAKE),
    IRtoPR("FR", "PR", Action.SCORE),
    OtoIR("O", "FR", Action.INTAKE),
    PLtoIL("PL", "FL", Action.INTAKE),
    PRtoIR("PR", "FR", Action.INTAKE);

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
              if (Robot.ROBOT_MODE != RobotMode.REAL)
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
      default: // this should never happen
        return Commands.none();
    }
  }

  // TODO aligning to climb pos correctly
  public Command climbPath(Path path, AutoRoutine routine) {
    // path align and climb
    return Commands.sequence(
        path.getTrajectory(routine)
            .cmd()
            .until(
                routine.observe(
                    path.getTrajectory(routine)
                        .atTime(
                            path.getTrajectory(routine).getRawTrajectory().getTotalTime()
                                - (0.3)))),
        setAutoClimbReqTrue());
  }

  public Command feedPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoFeedReqTrue(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        setAutoFeedReqFalse());
  }

  public Command scorePath(Path path, AutoRoutine routine) {
    // path align and score
    return Commands.sequence(
        path.getTrajectory(routine)
            .cmd()
            .until(
                routine.observe(
                    path.getTrajectory(routine)
                        .atTime(
                            path.getTrajectory(routine).getRawTrajectory().getTotalTime()
                                - (0.3)))),
        setAutoScoreReqTrue(),
        waitUntilEmpty(),
        setAutoScoreReqFalse());
  }

  // feeding and intake could prob be improved
  public Command intakePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoIntakeReqTrue(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()),
        setAutoIntakeReqFalse());
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

  // TODO: score at the start of each auto
  // specific paths:
  // no idea what to name them
  public Command getDepotScoreClimbAuto() {
    final AutoRoutine routine = factory.newRoutine("Depot Score Clim Auto");
    Path[] paths = {Path.PLtoD, Path.DtoIL, Path.ILtoILM, Path.ILMtoPL, Path.PLtoCL};
    // Will always need to reset odo at the start of a routine
    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry().andThen(shootPreload());
    // shoot preload then do the paths

    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }

    routine.active().whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command getOutpostScoreClimbAuto() {
    final AutoRoutine routine = factory.newRoutine("Outpost Score Climb Auto");
    Path[] paths = {Path.PRtoO, Path.OtoIR, Path.IRtoIRM, Path.IRMtoPR, Path.PRtoCR};
    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry().andThen(shootPreload());

    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }

    routine.active().whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command getDepotFeedClimbAuto() {
    final AutoRoutine routine = factory.newRoutine("Depot Feed Climb Auto");
    Path[] paths = {Path.PLtoD, Path.DtoFL, Path.FLtoFLM, Path.FLMtoPL, Path.PLtoCL};
    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry().andThen(shootPreload());

    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }

    routine.active().whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command getOutpostFeedClimbAuto() {
    final AutoRoutine routine = factory.newRoutine("Outpost Feed Climb Auto");
    Path[] paths = {Path.PLtoD, Path.DtoFL, Path.FLtoFLM, Path.FLMtoPL, Path.PLtoCL};
    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry().andThen(shootPreload());

    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }

    routine.active().whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command waitUntilEmpty() {
    // TODO wait till robot empty / done scoring
    // return null;
    return Commands.waitSeconds(0.5);
  }
}
