package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.MaplePhoenixUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class ModuleIOSim extends ModuleIOReal {

  private final SwerveModuleSimulation simulation;

  public ModuleIOSim(
      Module.ModuleConstants constants, SwerveModuleSimulation simulation, CANBus canbus) {
    super(constants, canbus);

    // Setup this modules simulation (lowk the only difference between Sim and Real except for not
    // using Async odo)
    this.simulation = simulation;
    this.simulation.useDriveMotorController(
        new MaplePhoenixUtil.TalonFXMotorControllerSim(
            driveTalon, SwerveSubsystem.SWERVE_CONSTANTS.getDriveMotorType(), true));
    this.simulation.useSteerMotorController(
        new MaplePhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(
            turnTalon,
            SwerveSubsystem.SWERVE_CONSTANTS.getTurnMotorType(),
            SwerveSubsystem.SWERVE_CONSTANTS.getTurnMotorInverted(),
            cancoder,
            false,
            Angle.ofBaseUnits(constants.cancoderOffset().getRadians(), Radian)));
  }
}
