package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.components.cancoder.CANcoderIO;

public class TurretIOSim extends TurretIO{
    TalonFXSimState motorSim;

  private final DCMotorSim physicsSim;

  private final double simLoopPeriod = 0.002; // 2 ms
  private Notifier simNotifier = null;
  private double lastSimTime = 0.0;


    public TurretIOSim(CANcoderIO can1, CANcoderIO can2) {
        super(can1, can2);
    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.01, TurretIO.TURRET_GEAR_RATIO),
            DCMotor.getKrakenX44Foc(1));

    motorSim = motor.getSimState();
    motorSim.setMotorType(MotorType.KrakenX44);
    motorSim.Orientation = ChassisReference.Clockwise_Positive;

    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

              physicsSim.setInputVoltage(motorSim.getMotorVoltage());
             physicsSim.update(deltaTime);

              motorSim.setRawRotorPosition(
                  physicsSim.getAngularPositionRad() * (TurretIO.TURRET_GEAR_RATIO));
              motorSim.setRotorVelocity(
                  physicsSim.getAngularVelocityRPM() * TurretIO.TURRET_GEAR_RATIO);
            });

    simNotifier.startPeriodic(simLoopPeriod);
  }
}
