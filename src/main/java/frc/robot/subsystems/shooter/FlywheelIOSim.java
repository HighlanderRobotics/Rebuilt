// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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

/** Add your docs here. */
public class FlywheelIOSim extends FlywheelIO {
  TalonFXSimState leaderFxSimState;
  DCMotorSim physicsSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(2), 0.0036, ShooterSubsystem.FLYWHEEL_GEAR_RATIO),
          DCMotor.getKrakenX60Foc(2));

  private final double simLoopPeriod = 0.002;
  private Notifier simNotifier;
  private double lastSimTime = 0.0;

  public FlywheelIOSim(TalonFXConfiguration config, CANBus canbus) {
    super(config, canbus);
    leaderFxSimState = flywheelLeader.getSimState();
    leaderFxSimState.setMotorType(MotorType.KrakenX60);
    leaderFxSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              leaderFxSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

              physicsSim.setInputVoltage(leaderFxSimState.getMotorVoltage());
              physicsSim.update(deltaTime);

              leaderFxSimState.setRawRotorPosition(
                  physicsSim.getAngularPosition().in(Rotations) * physicsSim.getGearing());
              leaderFxSimState.setRotorVelocity(
                  physicsSim.getAngularVelocity().in(RotationsPerSecond) * physicsSim.getGearing());
            });

    simNotifier.startPeriodic(simLoopPeriod);
  }
}
