// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public Rotation2d turretPositionRotations = new Rotation2d();
        public double turretAngularVelocity = 0.0;
        public double turretStatorCurrentAmps = 0.0;
        public double turretSupplyCurrentAmps = 0.0;
        public double turretVoltage = 0.0;
        public double turretTempC = 0.0;
        //_TODO: Input reall values 
    }
    protected TalonFX turretMotor;
    private final BaseStatusSignal turretPositionRotations;
    private final BaseStatusSignal turretAngularVelocity;
     private final StatusSignal<Voltage> turretVoltage;
     private final StatusSignal<Current> turretStatorCurrent;
     private final StatusSignal<Current> turretSupplyCurrent;
    private final StatusSignal<Temperature> turretTemp;
    
}
