// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  protected final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  protected final ShooterIOReal shooterIO;
  protected final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  protected final PivotIOReal pivotIO;
  protected final PivotIOInputsAutoLogged hoodInputs = new PivotIOInputsAutoLogged();
  protected final PivotIOReal hoodIO;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(ShooterIOReal shooterIO, PivotIOReal pivotIO, PivotIOReal hoodIO) {
    this.shooterIO = shooterIO;
    this.pivotIO = pivotIO;
    this.hoodIO = hoodIO;
  }

  public Command runStateCommand(
      Supplier<Rotation2d> pivotTarget,
      DoubleSupplier rollerVoltage,
      Supplier<Rotation2d> hoodTarget) {
    return this.run(
        () -> {
          Logger.recordOutput("Pivot Setpoint", pivotTarget.get());
          pivotIO.setMotorPosition(pivotTarget.get());
          Logger.recordOutput("Shooter Voltage", rollerVoltage.getAsDouble());
          shooterIO.setRollerVoltage(rollerVoltage.getAsDouble());
          Logger.recordOutput("Hood Setpoint", hoodTarget.get());
          hoodIO.setMotorPosition(hoodTarget.get());
        });
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Pivot", pivotInputs);
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Hood", hoodInputs);
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);
  }
}
