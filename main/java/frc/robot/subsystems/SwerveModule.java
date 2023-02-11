package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    
    private final CANSparkMax driveMotor;
    private final VictorSPX turningMotor;

    private final RelativeEncoder driveEncoder;
    private final edu.wpi.first.wpilibj.AnalogInput turningEncoder;

    private final PIDController turningPidController;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private double count = 0;

    public SwerveModule(int driveMotorID, int turningMotorId, boolean driveMotorReversed,
            boolean turningMotorReversed, int turningEncoderID, double absoluteEncoderOffset,
            boolean absoluteEncoderReversed){

                this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
                this.absoluteEncoderReversed = absoluteEncoderReversed;
                turningEncoder = new edu.wpi.first.wpilibj.AnalogInput(turningEncoderID);

                driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
                turningMotor = new VictorSPX(turningMotorId);

                turningMotor.configFactoryDefault();
                turningMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
                turningMotor.setSelectedSensorPosition(getTurningPosition() * 1658 / 180);
                //already initialized 
                //turningEncoder = new AnalogEncoder(turningEncoderID);

                driveMotor.setInverted(driveMotorReversed);
                turningMotor.setInverted(turningMotorReversed);

                driveEncoder = driveMotor.getEncoder();
                SmartDashboard.putNumber("offset", absoluteEncoderOffset);
                SmartDashboard.putNumber("turn position count", getTurningPosition());
                count = (getTurningPosition());
                SmartDashboard.putNumber("count", count);

                driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
                driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

                turningPidController = new PIDController(0.7, 0.2, 0.2);
                //turningPidController.enableContinuousInput(-Math.PI, Math.PI);

                resetEncoders();
            }   

            public double getDrivePosition(){
                return driveEncoder.getPosition();
            }

            public double getTurningPosition(){
                //return turningEncoder.getValue() / 36;
                double x = (((turningEncoder.getValue()) / 35.7365875));
                //if (x < 0) {
                //    x += 360;
                //}
                return (x - absoluteEncoderOffsetRad);
                //return (180 * ((turningEncoder.getValue() - count))/1024);
            }

            public double getDriveVelocity(){
                return driveEncoder.getVelocity();
            }

            public static double neoToMeters(double positionCounts, double circumference, double gearRatio) {
                return positionCounts * (circumference / (gearRatio *2048.0));
            }
            
            public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(
                    neoToMeters((driveEncoder.getPosition()), 43, ModuleConstants.kDriveMotorGearRatio),
                    new Rotation2d(getTurningPosition())
                );
            }

            //public double getTurningVelocity(){
                //return turningEncoder.getDistance(); -> Plus some math to figure out velocity
            //}

            //turningMotor.getSelectedSensorVelocity()
            //Not Needed?
            public double getTurningVelocity() {
                return turningMotor.getSelectedSensorVelocity();
            }

            //could be used to help write autonomous code and/or pathfinding/path smoothing math 
            //public double getAbsoluteEncoderRad(){
            //    double angle = turningEncoder.getAbsolutePosition();
            //    angle *= 2.0 * Math.PI;
            //    angle -= absoluteEncoderOffsetRad;
            //    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
            //}

            public void resetEncoders(){
                driveEncoder.setPosition(0);
                turningMotor.setSelectedSensorPosition(getTurningPosition());
                //have to call motor class not the encoder 
                //turningMotor.setSelectedSensorPosition(absoluteEncoder.getPosition());
            }

            public int getTicks() {
                return (int)turningMotor.getSelectedSensorPosition();
            }

            public double ticksToAngle(int ticks) {
                double angleTicks = ticks % 1658;

                double result = (angleTicks / (1658 / 2)) *180;

                if (result > 180) {
                    result -= 360;
                }

                return result;
            }

            public double getMeasuremnt() {
                return ticksToAngle(getTicks());
            }

            public SwerveModuleState getState(){
                return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
            }

            public void setDesiredState(SwerveModuleState state) {
                if (Math.abs(state.speedMetersPerSecond) < 0.001) {
                    stop();
                    return;
                }
                state = SwerveModuleState.optimize(state, getState().angle);
                driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
                //set turning motor mode to positional change 
                //this is the part that is not working most likely 
                SmartDashboard.putNumber("turning speed input", turningPidController.calculate(getTurningPosition(), state.angle.getDegrees()));
                //SmartDashboard.putNumber("turning position", getTurningPosition());
                turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(getTurningPosition() * 35.7365875, state.angle.getDegrees()));
                //turningMotor.set(ControlMode.PercentOutput, 0.3);
                //turningMotor.set(VictorSPXControlMode.PercentOutput, state.angle.getRadians());
                
            }

            public void stop() {
                driveMotor.set(0);
                resetEncoders();
                turningMotor.set(VictorSPXControlMode.Disabled, 0);
            }
}
