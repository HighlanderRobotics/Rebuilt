package frc.robot.components.rack;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.intake.LintakeSubsystem;

public class LinearRackIOSim extends LinearRackIO {
  // TODO: SHOULD THIS BE AN ELEVATOR?
  ElevatorSim physicsSim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              DCMotor.getKrakenX44Foc(1),
              Units.lbsToKilograms(10.0),
              LintakeSubsystem.RACK_PINION_DIAMETER_METERS / 2,
              LintakeSubsystem.RACK_GEAR_RATIO),
          DCMotor.getKrakenX44Foc(1),
          0.0,
          LintakeSubsystem.MAX_EXTENSION_METERS,
          false,
          0.0);

  private static final double SIM_LOOP_PERIOD = 0.002; // 2 ms
  private Notifier notifier;
  private TalonFXSimState talonSim;
  private double lastLoopTime = 0.0;

  public LinearRackIOSim(int motorId, CANBus canBus, TalonFXConfiguration config) {
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

              // I think these should be multiplied?
              talonSim.setRawRotorPosition(
                  physicsSim.getPositionMeters()
                      * (LintakeSubsystem.RACK_GEAR_RATIO
                          * (Math.PI * LintakeSubsystem.RACK_PINION_DIAMETER_METERS)));
              talonSim.setRotorVelocity(
                  physicsSim.getVelocityMetersPerSecond()
                      * (LintakeSubsystem.RACK_GEAR_RATIO
                          * (Math.PI * LintakeSubsystem.RACK_PINION_DIAMETER_METERS)));
            });

    notifier.startPeriodic(SIM_LOOP_PERIOD);
  }
}
