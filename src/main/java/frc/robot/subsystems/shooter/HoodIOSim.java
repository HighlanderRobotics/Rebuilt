package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

public class HoodIOSim extends HoodIO {
  TalonFXSimState hoodMotorSim;

  private final SingleJointedArmSim hoodPhysicsSim;

  // will get updated when i get specs

  private final double simLoopPeriod = 0.002; // 2 ms
  private Notifier simNotifier = null;
  private double lastSimTime = 0.0;

  public HoodIOSim(
      CANBus canbus,
      TalonFXConfiguration config,
      double gearRatio,
      int deviceID,
      Rotation2d minAngle,
      Rotation2d maxAngle) {
    super(config, canbus, deviceID);
    hoodPhysicsSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44Foc(1),
            gearRatio,
            // 0.5 * Units.lbsToKilograms(13.83) * Math.pow(Units.inchesToMeters(10), 2),
            0.1,
            Units.inchesToMeters(10),
            minAngle.getRadians(),
            maxAngle.getRadians(),
            true,
            minAngle.getRadians());
    // new DCMotorSim(
    //     LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.01, gearRatio),
    //     DCMotor.getKrakenX44Foc(1));

    hoodMotorSim = hoodMotor.getSimState();
    hoodMotorSim.setMotorType(MotorType.KrakenX44);
    hoodMotorSim.Orientation =
        config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    // hoodMotorSim.setRawRotorPosition(minAngle.getRotations() / gearRatio);

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
                  Units.radiansToRotations(hoodPhysicsSim.getAngleRads()) * (gearRatio));
              Logger.recordOutput("sim position rads", hoodPhysicsSim.getAngleRads() * gearRatio);
              hoodMotorSim.setRotorVelocity(
                  Units.radiansToRotations(hoodPhysicsSim.getVelocityRadPerSec()) * gearRatio);
              Logger.recordOutput(
                  "sim vel rads/s", hoodPhysicsSim.getVelocityRadPerSec() * gearRatio);
            });

    simNotifier.startPeriodic(simLoopPeriod);
  }
}
