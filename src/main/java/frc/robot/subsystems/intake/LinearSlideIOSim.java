package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class LinearSlideIOSim extends LinearSlideIO {
  // TODO: SHOULD THIS BE AN ELEVATOR?
  ElevatorSim physicsSim =
      new ElevatorSim(
          null, null, getSetpointMeters(), getSetpointMeters(), false, getSetpointMeters());

  private static final double SIM_LOOP_PERIOD = 0.002; // 2 ms
  private Notifier notifier;
  private TalonFXSimState talonSim;
  private double lastLoopTime = 0.0;

  public LinearSlideIOSim(int motorId, CANBus canBus, TalonFXConfiguration config) {
    super(motorId, canBus, config);

    this.talonSim = motor.getSimState();
    // Maybe try to make have these passed in? Maybe not needed tho
    this.talonSim.setMotorType(MotorType.KrakenX44);
    this.talonSim.Orientation = ChassisReference.Clockwise_Positive; // TODO

    notifier =
        new Notifier(
            () -> {
              double deltaTime = (Utils.getCurrentTimeSeconds() - lastLoopTime);
              lastLoopTime = Utils.getCurrentTimeSeconds();

              talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());

              physicsSim.setInputVoltage(talonSim.getMotorVoltage());
              physicsSim.update(deltaTime);

              talonSim.setRawRotorPosition(
                  physicsSim.getPositionMeters() * 1.0); // TODO: GEAR RATIO
              talonSim.setRotorVelocity(
                  physicsSim.getVelocityMetersPerSecond() * 1.0); // TODO: GEAR RATIO
            });

    notifier.startPeriodic(SIM_LOOP_PERIOD);
  }
}
