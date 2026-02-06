package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ExtensionIO {

  public static class ExtensionIOInputs {
    public double ExtensionVoltage = 0.0;
    public double ExtensionStatorCurrent = 0.0;
    public double ExtensionSupplyCurrent = 0.0;
    public double ExtensionTemp = 0.0;
  }
  ;

  public static final double GEAR_RATIO = 2.0;
  protected TalonFX Motor;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Voltage> ExtensionVoltage;
  private final StatusSignal<Current> ExtensionStatorCurrent;
  private final StatusSignal<Current> ExtensionSupplyCurrent;
  private final StatusSignal<Temperature> ExtensionTemp;

  public ExtensionIO(TalonFXConfiguration talonFXConfiguration, CANBus canbus, int deviceID) {
    Motor = new TalonFX(deviceID, canbus);
    ExtensionVoltage = Motor.getMotorVoltage();
    ExtensionStatorCurrent = Motor.getStatorCurrent();
    ExtensionSupplyCurrent = Motor.getSupplyCurrent();
    ExtensionTemp = Motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, ExtensionVoltage, ExtensionStatorCurrent, ExtensionSupplyCurrent, ExtensionTemp);
    Motor.optimizeBusUtilization();
  }

  public static TalonFXConfiguration getExtensionConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.Slot0.kS = 0.24;
    config.Slot0.kV = 0.6;
    config.Slot0.kP = 110.0;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }

  public void setExtensionVoltage(double voltage) {
    Motor.setControl(voltageOut.withOutput(voltage));
  }

  public void updateInputs(ExtensionIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        ExtensionVoltage, ExtensionStatorCurrent, ExtensionSupplyCurrent, ExtensionTemp);
  }
}
