package frc.robot.subsystems.turret;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim extends PivotIOReal {
  private final SingleJointedArmSim pivotPhysicsSim;
  TalonFXSimState motorSim;

  private static final double kSimLoopPeriod = 0.002; // 2 ms
  private Notifier simNotifier = null;
  private double lastSimTime = 0.0;

  public PivotIOSim(
      double minAngleRadians,
      double maxAngleRadians,
      double length,
      double maxVelocity,
      double maxAcceleration,
      TalonFXConfiguration config,
      int motorID,
      String name) {
    super(motorID, config, name);

    motorSim = motor.getSimState();
    motorSim.setMotorType(MotorType.KrakenX44);

    pivotPhysicsSim =
        new SingleJointedArmSim(
            new DCMotor(12.0, 4.05, 275, 1.4, 7530.0 / 60.0, 1),
            config.Feedback.SensorToMechanismRatio,
            0.1,
            length,
            minAngleRadians,
            maxAngleRadians,
            true,
            0.0);

    simNotifier =
        new Notifier(
            () -> {
              /* Calculate the time delta */
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* First set the supply voltage of all the devices */
              motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

              /* Then calculate the new position and velocity of the simulated elevator */
              pivotPhysicsSim.setInputVoltage(motorSim.getMotorVoltage());
              pivotPhysicsSim.update(deltaTime);

              /* Apply the new rotor position and velocity to the motors (before gear ratio) */
              motorSim.setRawRotorPosition(pivotPhysicsSim.getAngleRads()); // TODO gear ratio??
              // convert meters/second -> rotations/second
              motorSim.setRotorVelocity(
                  pivotPhysicsSim.getVelocityRadPerSec()); // TODO gear ratio??
            });
    simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public static PivotIOSim getTurretPivotSim() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kV = 0.0;
    config.Slot0.kG = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = 1.0;

    return new PivotIOSim((-1) * Math.PI / 2.0, Math.PI / 2.0, 1, 1, 1, config, 0, "Pivot");
  }

  public static PivotIOSim getTurretHoodSim() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kV = 0.0;
    config.Slot0.kG = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = 1.0;

    return new PivotIOSim(0, Units.degreesToRadians(10), 1, 1, 1, config, 1, "Hood");
  }
}
