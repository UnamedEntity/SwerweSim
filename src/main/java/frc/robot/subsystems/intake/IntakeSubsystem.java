// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Intake subsystem for collecting coral pieces */
public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO m_intakeIO;
  private final IntakeIO.IntakeIOInputs m_inputs = new IntakeIO.IntakeIOInputs();

  public IntakeSubsystem() {
      m_intakeIO = null; // Will be set in setSimulationIO method
      System.out.println("Intake: Using Maple-Sim Intake Simulation");
    
  }

  public void setSimulationIO(IntakeIO simIO) {
    if (RobotBase.isSimulation()) {
      try {
        var field = getClass().getDeclaredField("m_intakeIO");
        field.setAccessible(true);
        field.set(this, simIO);
        System.out.println("Intake: Simulation IO set successfully");
      } catch (Exception e) {
        System.err.println("Failed to set simulation IO: " + e.getMessage());
      }
    }
  }

  @Override
  public void periodic() {
    if (m_intakeIO != null) {
      m_intakeIO.updateInputs(m_inputs);
      
      // Log to SmartDashboard
      SmartDashboard.putBoolean("Intake/Note Detected", m_inputs.noteDetected);
      SmartDashboard.putNumber("Intake/Voltage", m_inputs.intakeVoltage);
    }
  }

  /** Set intake voltage directly */
  public void setIntakeVoltage(double voltage) {
      m_intakeIO.setIntakeVoltage(voltage);
  }

  /** Returns true if a coral piece is detected in the intake */
  public boolean isNoteDetected() {
    return m_inputs.noteDetected;
  }

  /** Returns true if a coral piece is detected in the intake */
  public boolean isNoteInsideIntake() {
    if (m_intakeIO != null) {
      return m_intakeIO.isNoteInsideIntake();
    }
    return false;
  }
}
