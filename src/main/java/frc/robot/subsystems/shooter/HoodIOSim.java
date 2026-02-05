package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim extends HoodIO {
  TalonFXSimState hoodMotorSim;

  private final DCMotorSim hoodPhysicsSim;

  // will get updated when i get specs

  private final double simLoopPeriod = 0.002; // 2 ms
  private Notifier simNotifier = null;
  private double lastSimTime = 0.0;

  public HoodIOSim(CANBus canbus, TalonFXConfiguration config, double gearRatio, int deviceID) {
    super(config, canbus, deviceID);
    hoodPhysicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.01, gearRatio),
            DCMotor.getKrakenX44Foc(1));

    hoodMotorSim = hoodMotor.getSimState();
    hoodMotorSim.setMotorType(MotorType.KrakenX44);
    hoodMotorSim.Orientation = ChassisReference.Clockwise_Positive;

    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              hoodMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

              hoodPhysicsSim.setInputVoltage(hoodMotorSim.getMotorVoltage());
              hoodPhysicsSim.update(deltaTime);

              // rotor position stuff added later when i have access to onshape

              hoodMotorSim.setRawRotorPosition(
                  hoodPhysicsSim.getAngularPositionRad() * (gearRatio));
              hoodMotorSim.setRotorVelocity(
                  hoodPhysicsSim.getAngularVelocityRPM() / 60.0 * gearRatio);
            });

    simNotifier.startPeriodic(simLoopPeriod);
  }
}
