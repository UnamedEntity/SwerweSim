// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

/** Command to run the intake motor in reverse to eject coral pieces */
public class OuttakeCommand extends Command {
  private final IntakeSubsystem m_intake;

  public OuttakeCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    // Intake is already running in reverse from initialize()
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    // This command runs until interrupted
    return false;
  }
}
