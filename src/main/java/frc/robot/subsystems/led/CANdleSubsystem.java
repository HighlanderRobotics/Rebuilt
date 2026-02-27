// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.candle.CANdleIO;
import frc.robot.components.candle.CANdleIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class CANdleSubsystem extends SubsystemBase {

  private final CANdleIO io;
  private final CANdleIOInputsAutoLogged inputs = new CANdleIOInputsAutoLogged();

  /** Creates a new CANdleSubsystem. */
  public CANdleSubsystem(CANdleIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CANdle", inputs);
  }

  public Command test() {
    return this.run(() -> io.setSolid(new RGBWColor(LEDSubsystem.PURPLE)));
  }

  public static CANdleConfiguration getCandleConfig() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    return config;
  }
}
