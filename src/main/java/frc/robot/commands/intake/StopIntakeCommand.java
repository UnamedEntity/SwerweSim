// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

/** Command to stop the intake motor */
public class StopIntakeCommand extends Command {
  private final IntakeSubsystem m_intake;

  public StopIntakeCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.stopIntake();
  }

  @Override
  public void execute() {
    // Intake is already stopped from initialize()
  }

  @Override
  public void end(boolean interrupted) {
    // Already stopped
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
