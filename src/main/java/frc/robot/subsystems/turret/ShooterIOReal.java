package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class ShooterIOReal {
  @AutoLog
  public static class ShooterIOInputs {
    public double velocityRotsPerSec = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
    public double motorTemperatureCelsius = 0.0;
  }

  protected final TalonFX leader;
  protected final TalonFX firstFollower;
  protected final TalonFX secondFollower;
  protected final TalonFX thirdFollower;

  // TODO find moi and gearing
  protected static double moi = 1.0;
  protected static double gearing = 1.0;

  private final VelocityVoltage velocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

  private final StatusSignal<AngularVelocity> angularVelocityRotsPerSec;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Temperature> motorTemperatureCelsius;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  public ShooterIOReal(
      int leaderID,
      int firstFollowerID,
      int secondFollowerID,
      int thirdFollowerID,
      TalonFXConfiguration config) {
    leader = new TalonFX(leaderID, "*");
    firstFollower = new TalonFX(firstFollowerID, "*");
    secondFollower = new TalonFX(secondFollowerID, "*");
    thirdFollower = new TalonFX(thirdFollowerID, "*");

    angularVelocityRotsPerSec = leader.getVelocity();
    supplyCurrentAmps = leader.getSupplyCurrent();
    appliedVoltage = leader.getMotorVoltage();
    statorCurrentAmps = leader.getStatorCurrent();
    motorTemperatureCelsius = leader.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        angularVelocityRotsPerSec,
        supplyCurrentAmps,
        statorCurrentAmps,
        appliedVoltage,
        motorTemperatureCelsius);

    leader.getConfigurator().apply(config);

    firstFollower.getConfigurator().apply(config);
    firstFollower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));

    secondFollower.getConfigurator().apply(config);
    secondFollower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));

    thirdFollower.getConfigurator().apply(config);
    thirdFollower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));

    leader.optimizeBusUtilization();
    firstFollower.optimizeBusUtilization();
    secondFollower.optimizeBusUtilization();
    thirdFollower.optimizeBusUtilization();
  }

  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRotsPerSec,
        supplyCurrentAmps,
        appliedVoltage,
        statorCurrentAmps,
        motorTemperatureCelsius);

    inputs.velocityRotsPerSec = angularVelocityRotsPerSec.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.motorTemperatureCelsius = motorTemperatureCelsius.getValueAsDouble();
  }

  public void setRollerVoltage(double volts) {
    leader.setControl(voltageOut.withOutput(volts));
  }

  public void setRollerVelocity(double velocityRPS) {
    leader.setControl(velocityVoltage.withVelocity(velocityRPS));
  }

  public static ShooterIOReal getShooterReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kP = 0.0;

    config.Feedback.SensorToMechanismRatio = 1.0;

    return new ShooterIOReal(12, 13, 14, 15, config);
  }
}
