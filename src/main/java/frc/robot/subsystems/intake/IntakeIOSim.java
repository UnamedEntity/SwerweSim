// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

/** Intake simulation implementation for Maple-Sim */
public class IntakeIOSim implements IntakeIO {
  private final IntakeSimulation intakeSimulation;
  private double m_intakeVoltage = 0.0;
  // subsystems/intake/IntakeIOSim.java

  public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
    // Create a coral intake simulation - mounted on the back side of the robot
    // This is an "Over-The-Bumper" intake that extends beyond the robot frame
    intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        // Specify the type of game pieces that the intake can collect
        // The game piece type must match what ReefscapeCoralOnField uses internally
        "Coral", // Game piece type for Reefscape 2025 coral
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
    
    // Note: IntakeSimulation is automatically registered with SimulatedArena when attached to drivetrain
    System.out.println("Intake simulation created for Coral game pieces");
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    int gamePieceCount = intakeSimulation.getGamePiecesAmount();
    inputs.noteDetected = gamePieceCount > 0;
    inputs.intakeVoltage = m_intakeVoltage;
    
    // Debug output when coral is detected
    if (gamePieceCount > 0 && m_intakeVoltage > 0) {
      System.out.println("Intake: Coral detected! Count: " + gamePieceCount);
    }
  }

  @Override
  public void setIntakeVoltage(double volts) {
    m_intakeVoltage = volts;
    // Convert voltage to intake state
    boolean running = Math.abs(volts) > 0.1;
    if (running) {
      intakeSimulation.startIntake(); 
    } else {
      intakeSimulation.stopIntake();
    }
  }
  public void stopIntake() {
    m_intakeVoltage = 0.0;
    intakeSimulation.stopIntake();
    System.out.println("Intake: Stopped ");
  }
  public void startIntake() {
    m_intakeVoltage = 6.0;
    intakeSimulation.startIntake();
    System.out.println("Intake: Started");
  }
  @Override
  public void setRunning(boolean runIntake) {
    if (runIntake) {
      m_intakeVoltage = 6.0; // Set positive voltage for intake
      intakeSimulation.startIntake(); // Extends the intake and starts detecting contacts
      System.out.println("Intake: Started");
    } else {
      m_intakeVoltage = 0.0; // Set voltage to zero
      intakeSimulation.stopIntake(); // Retracts the intake
      System.out.println("Intake: Stopped ");
    }
  }

  @Override
  public boolean isNoteInsideIntake() {
    return intakeSimulation.getGamePiecesAmount() != 0;
  }
  
  /**
   * Get the IntakeSimulation instance for direct access.
   * This allows other subsystems to check if coral has been collected.
   * 
   * @return The IntakeSimulation instance
   */
  public IntakeSimulation getIntakeSimulation() {
    return intakeSimulation;
  }
  
  /**
   * Add coral to the intake simulation.
   * This method adds a coral game piece to the field and integrates it with the intake simulation.
   * 
   * @param pose The pose where the coral should be placed on the field
   * @return The created ReefscapeCoralOnField instance
   */
  public ReefscapeCoralOnField addCoral(Pose2d pose) {
    // Create coral and add it to the simulation arena
    // The IntakeSimulation will automatically detect collisions with coral of type "Coral"
    ReefscapeCoralOnField coral = new ReefscapeCoralOnField(pose);
    org.ironmaple.simulation.SimulatedArena.getInstance().addGamePiece(coral);
    System.out.println("Coral added to intake simulation at pose: " + pose);
    return coral;
  }
  
  /**
   * Add coral to the intake simulation at a specific position.
   * 
   * @param x X coordinate in meters
   * @param y Y coordinate in meters
   * @param rotationDegrees Rotation in degrees
   * @return The created ReefscapeCoralOnField instance
   */
  public ReefscapeCoralOnField addCoral(double x, double y, double rotationDegrees) {
    return addCoral(new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees)));
  }
}
