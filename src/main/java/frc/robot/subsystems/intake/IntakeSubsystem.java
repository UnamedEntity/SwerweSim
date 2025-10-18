// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Intake subsystem for collecting coral pieces */
public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO m_intakeIO;
  private final IntakeIO.IntakeIOInputs m_inputs = new IntakeIO.IntakeIOInputs();

  public IntakeSubsystem() {
    // Create appropriate intake implementation based on simulation vs real robot
      // For simulation, we'll create a placeholder that will be replaced
      m_intakeIO = null; // Will be set in setSimulationIO method
      System.out.println("Intake: Using Maple-Sim Intake Simulation");
    
  }

  /** Set the simulation IO implementation (called from RobotContainer) */
  public void setSimulationIO(IntakeIO simIO) {
    if (RobotBase.isSimulation()) {
      // Use reflection to set the private field - not ideal but works for this case
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

  /** Start the intake motor to collect coral pieces */
  public void startIntake() {
    System.out.println("IntakeSubsystem.startIntake() called, m_intakeIO is " + (m_intakeIO != null ? "not null" : "NULL"));
    if (m_intakeIO != null) {
      m_intakeIO.setRunning(true);
      System.out.println("Intake: Set running to true");
    } else {
      System.out.println("Intake: ERROR - m_intakeIO is null!");
    }
  }

  /** Stop the intake motor */
  public void stopIntake() {
    if (m_intakeIO != null) {
      m_intakeIO.setRunning(false);
    }
  }

  /** Reverse the intake motor to eject coral pieces */
  public void reverseIntake() {
    System.out.println("IntakeSubsystem.reverseIntake() called, m_intakeIO is " + (m_intakeIO != null ? "not null" : "NULL"));
    if (m_intakeIO != null) {
      m_intakeIO.setIntakeVoltage(-IntakeConstants.kIntakeVoltage);
      System.out.println("Intake: Set voltage to " + (-IntakeConstants.kIntakeVoltage));
    } else {
      System.out.println("Intake: ERROR - m_intakeIO is null!");
    }
  }

  /** Set intake voltage directly */
  public void setIntakeVoltage(double voltage) {
    if (m_intakeIO != null) {
      m_intakeIO.setIntakeVoltage(voltage);
    }
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

  /** Eject coral piece (simulation only) */
  public boolean ejectCoral() {
    if (RobotBase.isSimulation() && m_intakeIO instanceof IntakeIOSim) {
      return ((IntakeIOSim) m_intakeIO).ejectCoral();
    }
    return false;
  }
}
