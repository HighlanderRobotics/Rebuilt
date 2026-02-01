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

public class TurretIOSim extends TurretIO{
    DCMotorSim turretphysicssSim =
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.001, TurretSubsystem.TURRET_GEAR_RATIO), 
        DCMotor.getKrakenX44Foc(1));
    
TalonFXSimState motorSim;
final double simLoopPeriod = 0.002;
Notifier simNotifier;
double lastSimTime = 0.0;

public TurretIOSim(TalonFXConfiguration configuration, CANBus canBus){
    super(configuration, canBus);
motorSim = turretMotor.getSimState();
motorSim.setMotorType(MotorType.KrakenX44);
motorSim.Orientation = ChassisReference.Clockwise_Positive;
simNotifier = new Notifier(() -> {
    double currentTime = Utils.getCurrentTimeSeconds();
    double deltaTime = currentTime - lastSimTime;
    lastSimTime = currentTime;
motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
turretphysicssSim.setInputVoltage(motorSim.getMotorVoltage());
turretphysicssSim.update(deltaTime);
motorSim.setRawRotorPosition(turretphysicssSim.getAngularPositionRotations() * turretphysicssSim.getGearing());
motorSim.setRotorVelocity(turretphysicssSim.getAngularVelocityRPM()/60 * turretphysicssSim.getGearing());

});
simNotifier.startPeriodic(simLoopPeriod);

}
}
