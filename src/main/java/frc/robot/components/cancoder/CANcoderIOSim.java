package frc.robot.components.cancoder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.math.geometry.Rotation2d;

public class CANcoderIOSim extends CANcoderIO {
  private final CANcoderSimState sim;

  private double positionRotations;

  public CANcoderIOSim(int cancoderID, CANcoderConfiguration config, CANBus canbus) {
    super(cancoderID, config, canbus);
    this.sim = cancoder.getSimState();
  }

  public void setSimValues(double positionRotations) {
    this.positionRotations = positionRotations;
  }

  public void updateInputs(CANcoderIOInputs inputs) {
    sim.setRawPosition(positionRotations);

    inputs.cancoderPositionRotations = Rotation2d.fromRotations(positionRotations);
  }
}
