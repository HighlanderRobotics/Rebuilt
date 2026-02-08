package frc.robot.components.cancoder;

import edu.wpi.first.wpilibj2.command.Commands;

public class CANcoderIOSim implements CANcoderIO {
  public CANcoderIOSim() {}

  public void updateInputs(CANcoderIOInputs inputs) {
    Commands.none();
  }
}
