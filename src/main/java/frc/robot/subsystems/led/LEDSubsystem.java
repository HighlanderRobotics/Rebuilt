// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Superstructure;
import frc.robot.Superstructure.SuperState;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  public static final int LED_LENGTH = 1; // TODO tbd
  public static final int LED_ID = 0;

  public static final Color PURPLE = new Color("#A000D0");

  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();
  private double dashStart = 0;

  // TODO add at extension supplier if relevant

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(LEDIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);
  }

  // public void addCommands(){
  // newTrigger(SuperState).whileTrue();

  private void solid(Color color) {
    io.solid(color);
  }

  private void setIndex(int i, Color color) {
    io.set(i, color);
  }

  private void solid(Color color, int start, int end) {
    for (int i = start; i < end; i++) {
      setIndex(i, color);
    }
  }

  public void blink(Color onColor, Color offColor, double duration) {
    blink(onColor, offColor, duration, 0, LED_LENGTH);
  }

  public void blink(Color onColor, Color offColor, double duration, int start, int end) {
    if (Timer.getTimestamp() % (2 * duration) < duration) {
      solid(onColor, start, end);
    } else {
      solid(offColor, start, end);
    }
  }

  public void runAlong(Color colorDash, Color colorBg, int dashLength, double frequency) {
    solid(colorBg);
    for (int i = (int) dashStart; i < dashStart + dashLength; i++) {
      setIndex(i % LED_LENGTH, colorDash);
    }
    dashStart += LED_LENGTH * frequency * 0.020;
    dashStart %= LED_LENGTH;
  }

  // methods to commands
  public Command solidCmd(Color color) {
    return this.run(() -> solid(color));
  }

  public Command solidRangeCmd(Color color, int start, int end) {
    return this.run(() -> solid(color, start, end));
  }

  public Command blinkCmd(Color onColor, Color offColor, double duration) {
    return Commands.run(() -> blink(onColor, offColor, duration)).repeatedly();
  }

  public Command blinkInRangeCmd(
      Color onColor, Color offColor, double duration, int start, int end) {
    return Commands.run(() -> blink(onColor, offColor, duration, start, end), this).repeatedly();
  }

  public Command runAlongCmd(Color colorDash, Color colorBg, int dashLength, double frequency) {
    return Commands.run(() -> runAlong(colorDash, colorBg, dashLength, frequency), this)
        .repeatedly();
  }

  public Command stateColorCmd(SuperState state) {
    return this.run(
        () -> {
          if (state == SuperState.INTAKE) {
            solidRangeCmd(Color.kGreen, 0, 50);
          } else if (Superstructure.isShootState()) {
            solidRangeCmd(Color.kRed, 0, 50);
          } else {
            solidRangeCmd(Color.kPurple, 0, 50);
          }
          if (state == SuperState.SCORE) {
            solidRangeCmd(Color.kBlue, 51, 100);
          } else if (state == SuperState.FEED) {
            solidRangeCmd(Color.kYellow, 51, 100);
          }
        });
  }
}

/*
Intake = green
Shoot = red
Score = blue
Feed = yellow
*/
