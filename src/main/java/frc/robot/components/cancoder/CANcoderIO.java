// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.cancoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

// for cancoders that aren't on a swerve module (eg arm, intake)
public class CANcoderIO {

  @AutoLog
  public static class CANcoderIOInputs {
    public boolean connected = false;
    public Rotation2d cancoderPositionRotations = new Rotation2d();
  }

  protected final CANcoder cancoder;

  private final StatusSignal<Angle> cancoderAbsolutePositionRotations;

  public CANcoderIO(int cancoderID, CANcoderConfiguration config, CANBus canbus) {
    cancoder = new CANcoder(cancoderID, canbus);
    cancoderAbsolutePositionRotations = cancoder.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, cancoderAbsolutePositionRotations);
    cancoder.getConfigurator().apply(config);
    cancoder.optimizeBusUtilization();
  }

  public void updateInputs(CANcoderIOInputs inputs) {
    BaseStatusSignal.refreshAll(cancoderAbsolutePositionRotations);

    inputs.connected = BaseStatusSignal.isAllGood(cancoderAbsolutePositionRotations);
    inputs.cancoderPositionRotations =
        Rotation2d.fromRotations(cancoderAbsolutePositionRotations.getValueAsDouble());
  }
}
