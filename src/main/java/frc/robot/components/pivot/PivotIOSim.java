package frc.robot.components.pivot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim extends PivotIO {
  private TalonFXSimState motorSimState;

  private final SingleJointedArmSim physicsSim;

  private final double simLoopPeriod = 0.002; // 2 ms
  private Notifier notifier = null;
  private double lastLoopTime = 0.0;

  public PivotIOSim(
      int motorId,
      TalonFXConfiguration config,
      CANBus canBus,
      SingleJointedArmSim physicsSim,
      MotorType motorType,
      double gearing) {
    super(motorId, config, canBus);
    this.motorSimState = motor.getSimState();
    this.motorSimState.setMotorType(motorType);

    this.physicsSim = physicsSim;

    notifier =
        new Notifier(
            () -> {
              double deltaTime = (Utils.getCurrentTimeSeconds() - lastLoopTime);
              lastLoopTime = Utils.getCurrentTimeSeconds();
              motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
              physicsSim.setInputVoltage(motorSimState.getMotorVoltage());
              physicsSim.update(deltaTime);
              motorSimState.setRawRotorPosition(
                  Units.radiansToRotations(physicsSim.getAngleRads()) * gearing);
              motorSimState.setRotorVelocity(
                  Units.radiansToRotations(physicsSim.getVelocityRadPerSec()) * gearing);
            });
    notifier.startPeriodic(simLoopPeriod);
  }
}
