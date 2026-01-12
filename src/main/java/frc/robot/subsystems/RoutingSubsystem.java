package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO!!!
public class RoutingSubsystem extends SubsystemBase {
    
    // TODO: BASE ON ACTUAL SENSOR READINGS
    private boolean isFull;
    private boolean isEmpty;

    public boolean isFull() {
        return isFull;
    }

    public boolean isEmpty() {
        return isEmpty;
    }
}
