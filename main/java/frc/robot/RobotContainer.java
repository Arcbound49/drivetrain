// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.swerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final swerveSubsystem swerveSubsystem = new swerveSubsystem();

  private final PS4Controller controller = new PS4Controller(0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  //this.fieldOriented = false;
        //this.limiter = new SlewRateLimiter(0.5, -0.5, 0);
        //this.turnLimiter = new SlewRateLimiter(0.4, -0.4, 0);
        //this.controller = new PS4Controller(0);
        //
        //this.turningSpeed = controller.getRightX();
        //this.xSpeed = controller.getLeftX();
        //this.ySpeed = controller.getLeftY();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem, 
    () -> controller.getLeftX(), 
    () -> controller.getLeftY(), 
    () -> controller.getRawAxis(4), false,
    () -> controller.getCircleButtonPressed(),
    () -> controller.getCrossButtonPressed(),
    () -> controller.getSquareButtonPressed(),
    () -> controller.getTriangleButtonPressed()));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveConstants.kPhysicalMaxSpeedMetersPerSecond, 
    DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond).setKinematics(DriveConstants.kDriveKinematics);
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(
      new Translation2d(1, 0),
      new Translation2d(1,-1)), 
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)), trajectoryConfig);

      PIDController xController = new PIDController(1.5, 0, 0);
      PIDController yController = new PIDController(1.5, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond));

      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);

      return new SequentialCommandGroup(new InstantCommand(() -> swerveSubsystem.resetOdometer(trajectory.getInitialPose())), swerveControllerCommand, new InstantCommand(() -> swerveSubsystem.stopModule()));
      
  }
}
