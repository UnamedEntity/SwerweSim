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

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems - use simulation in simulator, real hardware on robot
  private final DriveSubsystem m_robotDrive;
  private final IntakeSubsystem m_intake;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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
      // Get the drive simulation from the simulated drive subsystem
      var driveSim = simDrive.getDriveSimulation();
      System.out.println("Got drive simulation: " + (driveSim != null ? "not null" : "NULL"));
      IntakeIOSim intakeSim = new IntakeIOSim(driveSim);
      System.out.println("Created IntakeIOSim: " + (intakeSim != null ? "not null" : "NULL"));
      m_intake.setSimulationIO(intakeSim);
      System.out.println("Set simulation IO on intake subsystem");
    } else {
      System.out.println("NOT setting up simulation IO - isSimulation=" + RobotBase.isSimulation() + 
                        ", isSimulatedDrive=" + (m_robotDrive instanceof SimulatedDriveSubsystem));
    }
    
    // Now that m_robotDrive is initialized, configure buttons
    configureButtonBindings();

    // Configure default commands with keyboard support
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> {
                double xSpeed = 1;
                double ySpeed = 1;
                double rot = 1;
                
                                // Check if we're using keyboard or controller
                // If any keyboard keys are pressed, use keyboard, otherwise use controller
                boolean usingKeyboard = false;

                // Forward/Backward (W/S keys)
                if (m_driverController.getYButton()) { // Y button = W key in sim
                    xSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
                    usingKeyboard = true;
                }
                if (m_driverController.getAButton()) { // A button = S key in sim
                    xSpeed = -DriveConstants.kMaxSpeedMetersPerSecond;
                    usingKeyboard = true;
                }

                // Left/Right strafing (A/D keys)
                if (m_driverController.getXButton()) { // X button = A key in sim
                    ySpeed = DriveConstants.kMaxSpeedMetersPerSecond;
                    usingKeyboard = true;
                }
                if (m_driverController.getBButton()) { // B button = D key in sim
                    ySpeed = -DriveConstants.kMaxSpeedMetersPerSecond;
                    usingKeyboard = true;
                }

                // Rotation (Q/E keys)
                if (m_driverController.getLeftBumper()) { // LB = Q key in sim
                    rot = ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond;
                    usingKeyboard = true;
                }
                if (m_driverController.getRightBumper()) { // RB = E key in sim
                    rot = -ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond;
                    usingKeyboard = true;
                }

                // If not using keyboard, use controller joysticks
                if (!usingKeyboard) {
                    xSpeed = -m_driverController.getLeftY() * DriveConstants.kMaxSpeedMetersPerSecond;
                    ySpeed = -m_driverController.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond;
                    rot = -m_driverController.getRightX() * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond;
                }

                // Intake controls using A and B buttons
                if (m_driverController.getAButton()) { // A button for intake
                    System.out.println("Intake: Starting intake (A button pressed)");
                    m_intake.startIntake();
                } else if (m_driverController.getBButton()) { // B button for outtake
                    System.out.println("Intake: Reversing intake (B button pressed)");
                    m_intake.reverseIntake();
                } else {
                    m_intake.stopIntake();
                }

                m_robotDrive.drive(xSpeed, ySpeed, rot, false);
                
                
                m_robotDrive.drive(xSpeed, ySpeed, rot, false);
            },
            m_robotDrive));
    
    // Set default command for intake subsystem (idle command)
    m_intake.setDefaultCommand(new StopIntakeCommand(m_intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Reset heading with Back/Select button (or Space in keyboard mode)
    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // Intake controls
    // Y button - intake coral pieces
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new IntakeCommand(m_intake));
    
    // X button - outtake coral pieces  
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new OuttakeCommand(m_intake));
    
    // Start button - stop intake
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new StopIntakeCommand(m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            Pose2d.kZero,
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, Rotation2d.kZero),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return Commands.sequence(
        new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
  }
}