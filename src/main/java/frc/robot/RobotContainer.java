// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SimulatedDriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

public class RobotContainer {
  private final DriveSubsystem m_robotDrive;
  private final IntakeSubsystem m_intake;
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    // Create appropriate drive subsystem based on whether we're running in simulation
    if (RobotBase.isSimulation()) {
      m_robotDrive = new SimulatedDriveSubsystem();
      System.out.println("Using Maple-Sim Swerve Drive Simulation");
    } else {
      m_robotDrive = new DriveSubsystem();
      System.out.println("Using Real Swerve Drive Hardware");
    }
    
    // Create intake subsystem
    m_intake = new IntakeSubsystem();
    
    // Set up simulation IO if needed
    if (RobotBase.isSimulation() && m_robotDrive instanceof SimulatedDriveSubsystem) {
      System.out.println("Setting up intake simulation IO...");
      SimulatedDriveSubsystem simDrive = (SimulatedDriveSubsystem) m_robotDrive;
      var driveSim = simDrive.getDriveSimulation();
      System.out.println("Got drive simulation: " + (driveSim != null ? "not null" : "NULL"));
      IntakeIOSim intakeSim = new IntakeIOSim(driveSim);
      System.out.println("Created IntakeIOSim: " + (intakeSim != null ? "not null" : "NULL"));
      m_intake.setSimulationIO(intakeSim);
      System.out.println("Set simulation IO on intake subsystem");
    }
    
    // Configure button bindings
    configureButtonBindings();

    // Configure default drive command - SIMPLIFIED, NO INTAKE CONTROL HERE
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> {
                // Start with zero speeds
                double xSpeed = 0;
                double ySpeed = 0;
                double rot = 0;

                // Get joystick inputs (dead-banded)
                double leftY = -m_driverController.getLeftY();
                double leftX = -m_driverController.getLeftX();
                double rightX = -m_driverController.getRightX();
                
                // Apply deadband
                final double kDeadband = 0.1;
                if (Math.abs(leftY) > kDeadband) {
                    xSpeed = leftY * DriveConstants.kMaxSpeedMetersPerSecond;
                }
                if (Math.abs(leftX) > kDeadband) {
                    ySpeed = leftX * DriveConstants.kMaxSpeedMetersPerSecond;
                }
                if (Math.abs(rightX) > kDeadband) {
                    rot = rightX * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond;
                }

                m_robotDrive.drive(xSpeed, ySpeed, rot, true); // Field-relative
            },
            m_robotDrive));
    
    // DO NOT set a default command for intake - let button commands control it
  }

  private void configureButtonBindings() {
    // Reset heading with Back button
    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // Intake controls - use A and B buttons
    // A button - intake coral pieces (hold to run)
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new IntakeCommand(m_intake))
        .onFalse(new StopIntakeCommand(m_intake));
    
    // B button - outtake coral pieces (hold to run)
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new OuttakeCommand(m_intake))
        .onFalse(new StopIntakeCommand(m_intake));
  }

  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            Pose2d.kZero,
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.kZero),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    return Commands.sequence(
        new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
  }
}