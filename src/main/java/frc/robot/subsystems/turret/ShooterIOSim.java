package frc.robot.subsystems.turret;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim extends ShooterIOReal {
  TalonFXSimState leaderSim;
  // Could be an array but i suspect they may need to be independently controlled later
  TalonFXSimState firstFollowerSim;
  TalonFXSimState secondFollowerSim;
  TalonFXSimState thirdFollowerSim;

  // TODO uhh??????
  private final DCMotorSim firstFlywheelSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), moi, gearing),
          DCMotor.getKrakenX60Foc(2));

  private static final double kSimLoopPeriod = 0.002; // 2 ms
  private Notifier simNotifier = null;
  private double lastSimTime = 0.0;

  public ShooterIOSim(
      double jKgMetersSquared,
      double gearRatio,
      int leaderID,
      int firstFollowerID,
      int secondFollowerID,
      int thirdFollowerID,
      TalonFXConfiguration config) {

    super(leaderID, firstFollowerID, secondFollowerID, thirdFollowerID, config);

    // TODO idk if these are going clockwise or how the belt config is lmao
    leaderSim = leader.getSimState();
    leaderSim.setMotorType(MotorType.KrakenX60);
    leaderSim.Orientation = ChassisReference.Clockwise_Positive;

    firstFollowerSim = firstFollower.getSimState();
    firstFollowerSim.setMotorType(MotorType.KrakenX60);
    firstFollowerSim.Orientation = ChassisReference.Clockwise_Positive;

    secondFollowerSim = secondFollower.getSimState();
    secondFollowerSim.setMotorType(MotorType.KrakenX60);
    secondFollowerSim.Orientation = ChassisReference.Clockwise_Positive;

    thirdFollowerSim = thirdFollower.getSimState();
    thirdFollowerSim.setMotorType(MotorType.KrakenX60);
    thirdFollowerSim.Orientation = ChassisReference.Clockwise_Positive;

    simNotifier =
        new Notifier(
            () -> {
              /* Calculate the time delta */
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* First set the supply voltage of all the devices */
              leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

              /* Then calculate the new position and velocity of the simulated elevator */
              firstFlywheelSim.setInputVoltage(leaderSim.getMotorVoltage());
              firstFlywheelSim.update(deltaTime);

              /* Apply the new rotor position and velocity to the motors (before gear ratio) */
              leaderSim.setRawRotorPosition(
                  firstFlywheelSim.getAngularPositionRad()); // TODO gear ratio and stuff
              // convert RAD/second -> rotations/second
              leaderSim.setRotorVelocity(
                  firstFlywheelSim.getAngularVelocityRadPerSec()); // TODO gear ratio and stuff
            });
    simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public static ShooterIOSim getShooterSim() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kP = 0.0;

    config.Feedback.SensorToMechanismRatio = 1.0;

    return new ShooterIOSim(moi, gearing, 0, 1, 2, 3, config);
  }
}
