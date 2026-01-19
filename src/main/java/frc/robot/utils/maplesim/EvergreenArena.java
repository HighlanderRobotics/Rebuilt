package frc.robot.utils.maplesim;

import org.ironmaple.simulation.SimulatedArena;

/**
 * A maple sim arena with no collisions
 */
public class EvergreenArena extends SimulatedArena{
    public EvergreenArena() {
      super(new FieldMap() {});
    }

    @Override
    public void placeGamePiecesOnField() {}
}
