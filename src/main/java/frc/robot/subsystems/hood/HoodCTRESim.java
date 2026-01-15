package frc.robot.subsystems.hood;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodCTRESim extends HoodIOReal {
  TalonFXSimState hoodMotorSim;

  private final SingleJointedArmSim hoodPhysicsSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1), 0, 0, 0, 0, 0, false, 0); // will get updated when i get specs

  private static final double kSimLoopPeriod = 0.002; // 2 ms
  private Notifier simNotifier = null;
  private double lastSimTime = 0.0;

  public HoodCTRESim() {
    super();
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
            });
    simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public void updateInputs(HoodIOInputs inputs) {
    super.updateInputs(inputs);
  }
}
