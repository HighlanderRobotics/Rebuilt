package frc.robot.subsystems.climber;

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

//unfinished

public class ClimberIOSim extends ClimberIO {
    TalonFXSimState climberMotorSim;
        
private final DCMotorSim climberPhysicsSim =
    new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(1), 0.01, ClimberSubsystem.GEAR_RATIO),
          DCMotor.getKrakenX44Foc(1));

private final double simLoopPeriod = 0.002;
private Notifier simNotifier = null;
private double lastSimTime = 0.0;

public HoodIOSim(CANBus canbus) {
    super(ClimberIO.getClimberConfiguration(), canbus);
    climberMotorSim = climberMotor.getSimState();
    climberMotorSim.setMotorType(MotorType.KrakenX60);
    

}


}
