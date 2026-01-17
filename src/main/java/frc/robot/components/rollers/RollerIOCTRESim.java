package frc.robot.components.rollers;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOCTRESim extends RollerIOReal {

  private final DCMotorSim rollerSim;
  private TalonFXSimState talonSim;
  private double lastLoopTime = 0.0;
  Notifier notifier;

  public RollerIOCTRESim(
      int motorID, TalonFXConfiguration config, DCMotorSim motorSim, MotorType motorType) {

    super(motorID, config);
    rollerSim = motorSim;
    talonSim = rollerMotor.getSimState();
    talonSim.setMotorType(motorType);

    notifier =
        new Notifier(
            () -> {
              double deltaTime = (Utils.getCurrentTimeSeconds() - lastLoopTime);
              lastLoopTime = Utils.getCurrentTimeSeconds();
              talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());
              rollerSim.setInputVoltage(talonSim.getMotorVoltage());
              rollerSim.update(deltaTime);
              talonSim.setRawRotorPosition(
                  rollerSim.getAngularPositionRotations() * rollerSim.getGearing());
              talonSim.setRotorVelocity(
                  (rollerSim.getAngularVelocityRPM() / 60) * rollerSim.getGearing());
            });
    notifier.startPeriodic(0.002);
  }
}
