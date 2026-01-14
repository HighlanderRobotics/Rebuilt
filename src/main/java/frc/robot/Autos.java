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
//import frc.robot.Autos.PathEndType;
import frc.robot.Robot.RobotType;
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
    public static Trigger autoFeedReq =
        new Trigger(() -> autoFeed).and(DriverStation::isAutonomous);

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
  //R/L for right and left. 
//P for park 
//C for climb. 
//S for scoreing pos. 
//F for feeding poses
PLtoCL("PL", "CL", Action.CLIMB),
PRtoCR("PR", "CR", Action.CLIMB),
PLtoSL("PL", "CL", Action.SCORE),
PRtoSR("PR", "CR", Action.SCORE),
SLtoCL("SL", "CL", Action.CLIMB),
SRtoCR("SR", "CR", Action.CLIMB),
SLtoFL("SL", "FL", Action.FEED),
SRtoFR("SR", "FR", Action.FEED),
FRtoFL("FR", "FL", Action.FEED),
FLtoFR("FL", "FR", Action.FEED);

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
              if (Robot.ROBOT_TYPE != RobotType.REAL)
                Logger.recordOutput(
                    "Choreo/Active Traj",
                    DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get().equals(Alliance.Blue)
                        ? traj.getPoses()
                        : traj.flipped().getPoses());
            });
  }

  // TODO write leave auto
  public Command leaveAuto() {
    final AutoRoutine routine = factory.newRoutine("Leave Auto");
    Path[] paths = {};

    Command autoCommand = Commands.none();

    for (Path path : paths) {
      autoCommand =
          autoCommand.andThen(
              Commands.print("Running path: " + path.toString()).andThen(runPath(path, routine)));
    }
    return routine.cmd();
  }

  public Command runPath(Path path, AutoRoutine routine) {
    Action action = path.action;
    switch (action) {
      default: // this should never happen
        return Commands.none();
    }
  }

  public Command setAutoIntakeReqTrue() {
    return Commands.runOnce(
        () -> {
          autoIntake= true;
        });
  }
  public Command setAutoIntakeReqFalse() {
    return Commands.runOnce(
        () -> {
          autoIntake = false;
        });
  }

  public Command setAutoScoreReqTrue() {
    return Commands.runOnce(
        () -> {
          autoScore = true;
        });
  }
  public Command setAutoScoreReqFalse() {
    return Commands.runOnce(
        () -> {
          autoScore = false;
        });
  }

  public Command setAutoFeedReqTrue() {
    return Commands.runOnce(
        () -> {
          autoFeed = true;
        });
  }
  public Command setAutoFeedReqFalse() {
    return Commands.runOnce(
        () -> {
          autoFeed = false;
        });
  }

  public Command setAutoClimbReqTrue() {
    return Commands.runOnce(
        () -> {
          autoClimb = true;
        });
  }
  public Command setAutoClimbReqFalse() {
    return Commands.runOnce(
        () -> {
          autoClimb = false;
        });
  }
//TODO other things: depot autos, waiting for balls to be intaked/shot etc, make auto traj in choreo, write for the actaul paths
}
