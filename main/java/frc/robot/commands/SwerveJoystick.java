package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Camera;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.swerveSubsystem;

public class SwerveJoystick extends CommandBase{
    private swerveSubsystem swerveSubsystem;

    public double xSpeed, ySpeed, turningSpeed;
    private boolean fieldOriented;
    private SlewRateLimiter xlimiter, ylimiter;
    private SlewRateLimiter turnLimiter;
    private Supplier<Double> xSpeedF, ySpeedF, turningSpeedF, cubeIntake, coneIntake;
    private Supplier<Boolean> armSupplierUp, armSupplierDown, coneOut, cubeOut, autoAngle;
    public SwerveJoystick(swerveSubsystem swerveSubsystem, Supplier<Double> xSpeedF, 
    Supplier<Double> ySpeedF, Supplier<Double> turningSpeedF, boolean fieldOriented,
    Supplier<Boolean> armSupplierUp, Supplier<Boolean> armSupplierDown,
    Supplier<Double> coneIntake, Supplier<Double> cubeIntake,
    Supplier<Boolean> coneOut, Supplier<Boolean> cubeOut,
    Supplier<Boolean> autoAngle) {
        //this.fieldOriented = false;
        //this.limiter = new SlewRateLimiter(0.5, -0.5, 0);
        //this.turnLimiter = new SlewRateLimiter(0.4, -0.4, 0);
        //this.controller = new PS4Controller(0);
        //
        //this.turningSpeed = controller.getRightX();
        //this.xSpeed = controller.getLeftX();
        //this.ySpeed = controller.getLeftY();
        this.autoAngle = autoAngle;
        this.cubeIntake = cubeIntake;
        this.coneIntake = coneIntake;
        this.cubeOut = cubeOut;
        this.coneOut = coneOut;
        this.fieldOriented = fieldOriented;
        this.armSupplierUp = armSupplierUp;
        this.armSupplierDown = armSupplierDown;
        this.xlimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.ylimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turnLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedF = xSpeedF;
        this.ySpeedF = ySpeedF;
        this.turningSpeedF = turningSpeedF;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        xSpeed = xSpeedF.get() / 3;
        ySpeed = ySpeedF.get() / 3;
        turningSpeed = (turningSpeedF.get());

        xSpeed = Math.abs(xSpeed) > OperatorConstants.deadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OperatorConstants.deadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OperatorConstants.deadband ? turningSpeed : 0.0;

        xSpeed = xlimiter.calculate(xSpeed) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        ySpeed = ylimiter.calculate(ySpeed) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        //get integer input of controller and using that as a percent of 2 pi to figure out how much to turn 
        turningSpeed = (turnLimiter.calculate(turningSpeed) * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond);
        xSpeed = SwerveModule.round(xSpeed, 3);
        ySpeed = SwerveModule.round(ySpeed, 3);
        turningSpeed = SwerveModule.round(turningSpeed, 3);

        ChassisSpeeds chassisSpeeds;
        if(fieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            SmartDashboard.putNumber("x", xSpeed);
            SmartDashboard.putNumber("y", ySpeed);
            SmartDashboard.putNumber("t", turningSpeed);
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }   

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        SmartDashboard.putString("states", moduleStates[0].toString());
        swerveSubsystem.setModuleStates(moduleStates);

        if(armSupplierUp.get() == true) {
            FourBar.MoveUp(0.3);
        } else if(armSupplierDown.get() == true) {
            FourBar.MoveDown(0.3);
        } else {
            FourBar.MoveDown(0);
            FourBar.MoveUp(0);
        }

        //if(coneIntake.get() >= 0.3) {
        //    EndEffector.coneIntake(0.5);
        //} else if (cubeIntake.get() >= 0.3) {
        //    EndEffector.cubeIntake(0.3);
        //} else if(coneOut.get() == true) {
        //    EndEffector.coneOut();
        //} else if (cubeOut.get() == true) {
        //    EndEffector.cubeOut();
        //} else {
        //    EndEffector.stop();
        //}

        if (autoAngle.get() == true) {
            Camera.softDriveTargeting();
        }
    }

    @Override
    public void end(boolean interrupted) {
        FourBar.Move(0);
        swerveSubsystem.stopModule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
