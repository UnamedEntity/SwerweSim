// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/**
 * Simulated swerve drive subsystem that extends DriveSubsystem.
 * Uses Maple-Sim physics simulation for realistic robot behavior in simulation mode.
 * This uses the SelfControlledSwerveDriveSimulation which handles motor control automatically.
 */
public class SimulatedDriveSubsystem extends DriveSubsystem {
  private final SelfControlledSwerveDriveSimulation simulatedDrive;
  private final Field2d field2d;

  /** Creates a new SimulatedDriveSubsystem. */
  public SimulatedDriveSubsystem() {
    super();
    
    // Configure the drivetrain simulation with your robot's physical parameters
    final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
        // Configure swerve module with your motors and gear ratios
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                DCMotor.getNEO(1),      // Drive motor (adjust if using different motors)
                DCMotor.getNeo550(1),   // Steer motor (adjust if using different motors)
                6.75,                   // Drive gear ratio (adjust to your actual ratio)
                12.8,                   // Steer gear ratio (adjust to your actual ratio)
                Units.Volts.of(0.1),    // Drive friction voltage
                Units.Volts.of(0.1),    // Steer friction voltage
                Units.Meters.of(ModuleConstants.kWheelDiameterMeters / 2.0), // Wheel radius
                Units.KilogramSquareMeters.of(0.025), // Steer moment of inertia
                1.0                     // Wheel coefficient of friction
            )
        )
        // Configure track dimensions from your constants
        .withTrackLengthTrackWidth(
            Units.Meters.of(DriveConstants.kWheelBase),
            Units.Meters.of(DriveConstants.kTrackWidth)
        )
        // Configure bumper size (adjust to your robot size in meters)
        .withBumperSize(
            Units.Meters.of(0.9),
            Units.Meters.of(0.9)
        );
    
    // Create the SwerveDriveSimulation
    SwerveDriveSimulation driveSimulation = new SwerveDriveSimulation(config, new Pose2d());
    
    // Wrap it in SelfControlledSwerveDriveSimulation for automatic motor control
    this.simulatedDrive = new SelfControlledSwerveDriveSimulation(driveSimulation);
    
    // Register to simulation world for physics updates
    SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());
    
    // Create field widget for visualization on dashboard
    field2d = new Field2d();
    SmartDashboard.putData("Simulation Field", field2d);
    
    System.out.println("Maple-Sim Swerve Drive Simulation initialized");
  }

  @Override
  public void periodic() {
    // Update the simulation - this handles motor control and odometry
    simulatedDrive.periodic();
    
    // Update parent class periodic (for any additional logic)
    super.periodic();
    
    // Update field visualization
    field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
    field2d.getObject("Odometry").setPose(simulatedDrive.getOdometryEstimatedPose());
    
    // Optional: Display simulation data on dashboard
    SmartDashboard.putNumber("Sim Heading", getHeading());
    SmartDashboard.putNumber("Sim X", getPose().getX());
    SmartDashboard.putNumber("Sim Y", getPose().getY());
  }

  @Override
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Use the simplified simulation API to command the drive
    simulatedDrive.runChassisSpeeds(
        new ChassisSpeeds(xSpeed, ySpeed, rot),
        new Translation2d(), // Center of rotation (robot center)
        fieldRelative,
        true  // Open loop control
    );
  }

  @Override
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Use the simplified simulation API to set module states
    simulatedDrive.runSwerveStates(desiredStates);
  }

  @Override
  public Pose2d getPose() {
    // Return the odometry-estimated pose (includes drift like real robot)
    return simulatedDrive.getOdometryEstimatedPose();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    // Reset both the simulation world pose and odometry
    simulatedDrive.setSimulationWorldPose(pose);
    simulatedDrive.resetOdometry(pose);
  }

  @Override
  public double getHeading() {
    // Get heading from odometry (this matches what real robot would use)
    return simulatedDrive.getOdometryEstimatedPose().getRotation().getDegrees();
  }

  @Override
  public void zeroHeading() {
    // Reset heading to zero
    Pose2d currentPose = simulatedDrive.getActualPoseInSimulationWorld();
    Pose2d newPose = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    simulatedDrive.setSimulationWorldPose(newPose);
    simulatedDrive.resetOdometry(newPose);
  }

  @Override
  public double getTurnRate() {
    // Get turn rate from simulated chassis speeds
    ChassisSpeeds speeds = simulatedDrive.getMeasuredSpeedsFieldRelative(true);
    return Math.toDegrees(speeds.omegaRadiansPerSecond);
  }

  /**
   * Gets the simulated chassis speeds for advanced control.
   * @return The measured chassis speeds from simulation
   */
  public ChassisSpeeds getMeasuredSpeeds() {
    return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
  }

  /**
   * Gets the actual simulated pose (ground truth).
   * Useful for comparing odometry accuracy.
   * @return The true simulated pose
   */
  public Pose2d getSimulatedPose() {
    return simulatedDrive.getActualPoseInSimulationWorld();
  }
}