// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class HoodIOReal {
  /** Creates a new HoodIOReal. */
  @AutoLog
  public class HoodIOInputs {
    public double hoodPositionMeters = 0.0;
    public double hoodVelocityMetersPerSecond = 0.0;
    public double hoodStatorCurrentAmps = 0.0;
    public double hoodSupplyCurrentAmp = 0.0;
    public double hoodVoltage = 0.0;
    public double hoodTempC = 0.0;
  }

  TalonFX hoodMotor = new TalonFX(1);

  private final BaseStatusSignal hoodPositionMeters = hoodMotor.getPosition();
  private final BaseStatusSignal hoodVelocityMetersPerSec = hoodMotor.getVelocity();
  private final StatusSignal<Voltage> hoodVoltage = hoodMotor.getMotorVoltage();
  private final StatusSignal<Current> hoodStatorCurrent = hoodMotor.getStatorCurrent();
  private final StatusSignal<Current> hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> hoodTemp = hoodMotor.getDeviceTemp();
  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

  public void setHoodVoltage(double hoodVoltage) {
    hoodMotor.setControl(voltageOut.withOutput(hoodVoltage));
  }

  public void setHoodPosition(Rotation2d hoodPosition) {
    hoodMotor.setControl(positionVoltage.withPosition(hoodPosition.getMeasure()));
  }

  public void setHoodVelocity(double hoodVelocity) {
    hoodMotor.setControl(velocityVoltage.withVelocity(hoodVelocity));
  }

  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        hoodPositionMeters,
        hoodVelocityMetersPerSec,
        hoodVoltage,
        hoodStatorCurrent,
        hoodSupplyCurrent,
        hoodTemp);

    inputs.hoodPositionMeters = hoodPositionMeters.getValueAsDouble();
    inputs.hoodVelocityMetersPerSecond = hoodVelocityMetersPerSec.getValueAsDouble();
    inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
    inputs.hoodStatorCurrentAmps = hoodStatorCurrent.getValueAsDouble();
    inputs.hoodSupplyCurrentAmp = hoodSupplyCurrent.getValueAsDouble();
    inputs.hoodTempC = hoodTemp.getValueAsDouble();
  }
}
