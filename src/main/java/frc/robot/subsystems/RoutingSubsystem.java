package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
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

    // TODO!!
    public Command index() {
        return this.run(() -> {});
    }

    // Can't call it idle bc idle is smthing else
    public Command rest() {
        // TODO
        return idle();
    }

    public Command reverseIndex() {
        // TODO
        return this.run(() -> {});
    }
}
