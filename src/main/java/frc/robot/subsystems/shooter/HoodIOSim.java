package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
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

  private final DCMotorSim hoodPhysicsSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(1), 0.01, ShooterSubsystem.HOOD_GEAR_RATIO_A),
          DCMotor.getKrakenX44Foc(1));

  // will get updated when i get specs

  private final double simLoopPeriod = 0.002; // 2 ms
  private Notifier simNotifier = null;
  private double lastSimTime = 0.0;

  public HoodIOSim(CANBus canbus) {
    super(HoodIO.getHoodAlphaConfiguration(), canbus);
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
                  hoodPhysicsSim.getAngularPositionRad() * (ShooterSubsystem.HOOD_GEAR_RATIO_A));
              hoodMotorSim.setRotorVelocity(
                  hoodPhysicsSim.getAngularVelocityRPM() / 60.0 * ShooterSubsystem.HOOD_GEAR_RATIO_A);
            });
    simNotifier.startPeriodic(simLoopPeriod);
  }
}
