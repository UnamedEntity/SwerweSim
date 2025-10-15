// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.IntakeConstants;

/** Intake hardware implementation for real robot */
public class IntakeIORobot implements IntakeIO {
  private final PWMMotorController m_intakeMotor;
  private final DigitalInput m_noteSensor;

  public IntakeIORobot() {
    m_intakeMotor = new Spark(IntakeConstants.kIntakeMotorPort);
    m_noteSensor = new DigitalInput(IntakeConstants.kNoteSensorPort);
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    inputs.noteDetected = !m_noteSensor.get(); // Inverted because sensor is normally closed
    inputs.intakeVoltage = m_intakeMotor.get();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    m_intakeMotor.set(volts);
  }

  @Override
  public void setRunning(boolean running) {
    setIntakeVoltage(running ? IntakeConstants.kIntakeVoltage : 0.0);
  }

  @Override
  public boolean isNoteInsideIntake() {
    return !m_noteSensor.get(); // Inverted because sensor is normally closed
  }
}
