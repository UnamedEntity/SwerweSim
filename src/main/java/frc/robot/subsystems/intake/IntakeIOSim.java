// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import edu.wpi.first.units.Units;
import frc.robot.Constants.IntakeConstants;

/** Intake simulation implementation for Maple-Sim */
public class IntakeIOSim implements IntakeIO {
  private final IntakeSimulation m_intakeSimulation;
  private double m_intakeVoltage = 0.0;

  public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
    // Create a coral intake simulation - mounted on the back side of the robot
    // This is an "Over-The-Bumper" intake that extends beyond the robot frame
    m_intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        // Specify the type of game pieces that the intake can collect
        "Coral", // Using "Coral" as the game piece type for Reefscape 2025
        // Specify the drivetrain to which this intake is attached
        driveTrain,
        // Width of the intake (coral pieces are typically wider than notes)
        Units.Meters.of(0.4), // 40cm wide intake for coral pieces
        // The extension length of the intake beyond the robot's frame (when activated)
        Units.Meters.of(0.15), // 15cm extension beyond bumper
        // The intake is mounted on the back side of the chassis
        IntakeSide.BACK,
        // The intake can hold up to 1 coral piece
        1
    );
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    inputs.noteDetected = m_intakeSimulation.getGamePiecesAmount() > 0;
    inputs.intakeVoltage = m_intakeVoltage;
  }

  @Override
  public void setIntakeVoltage(double volts) {
    m_intakeVoltage = volts;
    // Convert voltage to intake state
    boolean running = Math.abs(volts) > 0.1;
    if (running) {
      m_intakeSimulation.startIntake(); // Extends the intake and starts detecting contacts
    } else {
      m_intakeSimulation.stopIntake(); // Retracts the intake
    }
  }

  @Override
  public void setRunning(boolean running) {
    setIntakeVoltage(running ? IntakeConstants.kIntakeVoltage : 0.0);
  }

  @Override
  public boolean isNoteInsideIntake() {
    return m_intakeSimulation.getGamePiecesAmount() > 0;
  }

  /**
   * Eject the coral piece from the intake
   * @return true if a coral piece was ejected, false if intake was empty
   */
  public boolean ejectCoral() {
    return m_intakeSimulation.obtainGamePieceFromIntake();
  }
}
