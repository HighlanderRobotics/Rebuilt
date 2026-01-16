// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import com.google.common.base.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
  HoodIO io;
  HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public static double GEAR_RATIO = 147.0/13.0;
  

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem(HoodIO io) {
    this.io = io;
  }

  public Command setHoodPositionCommand(Supplier<Rotation2d> hoodPosition) {
    return this.run(() -> io.setHoodPosition(hoodPosition));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Hood", inputs);
  }
}
