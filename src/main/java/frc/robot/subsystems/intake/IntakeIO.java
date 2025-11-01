// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


public interface IntakeIO {
  public static class IntakeIOInputs {
    public boolean noteDetected = false;
    public double intakeVoltage = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets the intake motor voltage */
  public default void setIntakeVoltage(double volts) {}

  /** Sets whether the intake is running */
  public default void setRunning(boolean running) {}

  /** Returns true if a note is detected in the intake */
  public default boolean isNoteInsideIntake() {
    return false;
  }
}
