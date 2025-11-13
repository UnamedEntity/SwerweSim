// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;



public class SimulatedDriveSubsystem extends DriveSubsystem {
  private final SelfControlledSwerveDriveSimulation simulatedDrive;
  private final SwerveDriveSimulation baseSimulation;
  private final Field2d field2d;
  
  // Publishers for 3D visualization in AdvantageScope
  private final StructArrayPublisher<Pose3d> robotPosePublisher;
  private final StructArrayPublisher<Pose3d> gamePiecePublisher;
  
  // Reference to coral for physics-based pose tracking
  private ReefscapeCoralOnField coral;
  
  // Reference to intake simulation to check if coral has been collected
  private IntakeSimulation intakeSimulation;

  /** Creates a new SimulatedDriveSubsystem. */
  public SimulatedDriveSubsystem() {
    super();
    
    // Configure the drivetrain simulation with your robot's physical parameters
    final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                DCMotor.getNEO(1),      // Drive motor
                DCMotor.getNeo550(1),   // Steer motor
                6.75,                   // Drive gear ratio
                12.8,                   // Steer gear ratio
                Units.Volts.of(0.5),    // Drive friction voltage
                Units.Volts.of(0.5),    // Steer friction voltage
                Units.Meters.of(ModuleConstants.kWheelDiameterMeters / 2.0),
                Units.KilogramSquareMeters.of(0.025),
                1.0                     // Wheel coefficient of friction
            )
        )
        .withTrackLengthTrackWidth(
            Units.Meters.of(DriveConstants.kWheelBase),
            Units.Meters.of(DriveConstants.kTrackWidth)
        )
        .withBumperSize(
            Units.Meters.of(0.9),
            Units.Meters.of(0.9)
        );
    
    // Create the base SwerveDriveSimulation
    this.baseSimulation = new SwerveDriveSimulation(
        config, 
        new Pose2d(8.0, 4.0, new Rotation2d())
    );
    
    // Wrap it with SelfControlledSwerveDriveSimulation for automatic control
    this.simulatedDrive = new SelfControlledSwerveDriveSimulation(baseSimulation);
    
    // Register the base simulation to the world for physics updates
    SimulatedArena.getInstance().addDriveTrainSimulation(baseSimulation);
    
    // Create field widget for 2D visualization on dashboard
    field2d = new Field2d();
    SmartDashboard.putData("Simulation Field", field2d);
    
    // Setup 3D visualization publishers for AdvantageScope
    // Robot pose - standard topic that AdvantageScope expects
    robotPosePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("AdvantageScope/Robot", Pose3d.struct)
        .publish();
    
    // Game pieces publisher for AdvantageScope
    gamePiecePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("AdvantageScope/GamePieces", Pose3d.struct)
        .publish();
    
    // Spawn game pieces once at initialization - behind the robot
    // Robot starts at (8.0, 4.0) facing forward (0 degrees), intake is on BACK side
    // Place coral 1.5 meters behind the robot
    coral = new ReefscapeCoralOnField(new Pose2d(6.5, 4.0, Rotation2d.fromDegrees(90)));
    SimulatedArena.getInstance().addGamePiece(coral);
    
    // GamePieceContactListener is automatically set up by IntakeSimulation when it's created
    // with game piece type "Coral". The listener will remove coral from field when collision detected.
    
    SimulatedArena.getInstance().addGamePiece(
        new ReefscapeAlgaeOnField(new Translation2d(6.5, 4.0))
    );
    System.out.println("Game pieces spawned behind robot at (6.5, 4.0)");
  }

  @Override
  public void periodic() {
    // DO NOT call super.periodic() - it tries to use hardware encoders that don't exist
    
    // Update the simulated drive's odometry
    simulatedDrive.periodic();
    
    // Get poses (cached to avoid multiple calls)
    Pose2d actualPose = simulatedDrive.getActualPoseInSimulationWorld();
    Pose2d odometryPose = simulatedDrive.getOdometryEstimatedPose();
    
    // Update 2D field visualization
    field2d.setRobotPose(actualPose);
    field2d.getObject("odometry").setPose(odometryPose);
    
    // Publish 3D robot pose for AdvantageScope
    robotPosePublisher.set(new Pose3d[] {
        new Pose3d(
            actualPose.getX(), 
            actualPose.getY(), 
            0.0,
            new Rotation3d(0, 0, actualPose.getRotation().getRadians())
        )
    });
    
    // Check if coral has been collected by checking intake simulation directly
    // When coral is collected, IntakeSimulation.getGamePiecesAmount() will be > 0
    // and the GamePieceContactListener will have removed it from the field
    if (intakeSimulation != null && coral != null) {
      int gamePieceCount = intakeSimulation.getGamePiecesAmount();
      if (gamePieceCount > 0) {
        // Coral has been collected - remove it from visualization
        coral = null;
        System.out.println("Coral collected! Removing from visualization.");
      }
    }
    
    // Publish game pieces for AdvantageScope with physics-based poses
    // Coral has physics enabled through SimulatedArena.addGamePiece()
    // The coral's pose is updated by the physics engine during simulationPeriodic()
    // When coral is collected, it's removed from the visualization
    Pose3d[] gamePiecePoses;
    if (coral != null) {
      // Coral is still on the field - physics simulation is handling its position
      // The pose will be updated by the physics engine, but for visualization we use initial pose
      // TODO: Query coral's actual physics body pose for real-time updates
      gamePiecePoses = new Pose3d[] {
          new Pose3d(6.5, 4.0, 0.1, new Rotation3d(0, 0, Math.toRadians(90))), // Coral (physics-enabled)
          new Pose3d(6.5, 4.0, 0.1, new Rotation3d()) // Algae (static for now)
      };
    } else {
      // Coral has been collected and removed from field via GamePieceContactListener
      gamePiecePoses = new Pose3d[] {
          new Pose3d(6.5, 4.0, 0.1, new Rotation3d()) // Algae only
      };
    }
    gamePiecePublisher.set(gamePiecePoses);
    
    // Display essential data
    SmartDashboard.putNumber("Sim X", actualPose.getX());
    SmartDashboard.putNumber("Sim Y", actualPose.getY());
    SmartDashboard.putNumber("Sim Heading", actualPose.getRotation().getDegrees());
  }
  
  @Override
  public void simulationPeriodic() {
    // SimulatedArena handles physics updates automatically
    // This includes collision detection between robot and coral via GamePieceContactListener
    SimulatedArena.getInstance().simulationPeriodic();
  }

  @Override
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
      // Create chassis speeds
      ChassisSpeeds speeds = fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose().getRotation())
          : new ChassisSpeeds(xSpeed, ySpeed, rot);
  
      // Send the chassis speeds to the simulation
      simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), fieldRelative, false);
  }

  @Override
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Use the SelfControlledSwerveDriveSimulation's runSwerveStates method
    // This is used for autonomous trajectory following
    simulatedDrive.runSwerveStates(desiredStates);
  }

  @Override
  public Pose2d getPose() {
    // Return the odometry-estimated pose (what the robot thinks its position is)
    return simulatedDrive.getOdometryEstimatedPose();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    // Set both the simulation world pose and reset odometry
    simulatedDrive.setSimulationWorldPose(pose);
    simulatedDrive.resetOdometry(pose);
    System.out.println("Reset odometry to: " + pose);
  }

  @Override
  public void zeroHeading() {
    Pose2d currentPose = getPose();
    resetOdometry(new Pose2d(currentPose.getTranslation(), new Rotation2d()));
  }

  @Override
  public double getHeading() {
    return getPose().getRotation().getDegrees();
  }

  @Override
  public double getTurnRate() {
    ChassisSpeeds speeds = simulatedDrive.getMeasuredSpeedsFieldRelative(false);
    return Math.toDegrees(speeds.omegaRadiansPerSecond);
  }
  
  @Override
  public void resetEncoders() {
    // Not needed for simulation
  }

  /** Get the underlying drive simulation for other subsystems to use */
  public org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation getDriveSimulation() {
    return baseSimulation;
  }
  
  /**
   * Set the intake simulation reference to track when coral is collected.
   * This allows the drive subsystem to remove coral from visualization when collected.
   * 
   * @param intakeSim The IntakeSimulation instance
   */
  public void setIntakeSimulation(IntakeSimulation intakeSim) {
    this.intakeSimulation = intakeSim;
  }
}