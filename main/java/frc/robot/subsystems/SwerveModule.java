package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    
    private final CANSparkMax driveMotor;
    private final WPI_VictorSPX turningMotor;

    private final RelativeEncoder driveEncoder;
    private final edu.wpi.first.wpilibj.AnalogInput turningEncoder;

    private final ProfiledPIDController turningPidController;
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
                turningMotor = new WPI_VictorSPX(turningMotorId);

                turningMotor.configFactoryDefault();
                turningMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
                turningMotor.setSelectedSensorPosition(getTurningPosition() / 35.7365875 * (Math.pow(Math.PI, 2)));
                //already initialized 
                //turningEncoder = new AnalogEncoder(turningEncoderID);
                driveMotor.setInverted(driveMotorReversed);
                turningMotor.setInverted(turningMotorReversed);

                driveEncoder = driveMotor.getEncoder();

                driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
                driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

                turningPidController = new ProfiledPIDController(7, 0, 0, 
                new TrapezoidProfile.Constraints(DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, 
                DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond));
                //turningPidController.enableContinuousInput(-Math.PI, Math.PI);

                resetEncoders();
            }   

            public double getDrivePosition(){
                return driveEncoder.getPosition();
            }

            public double getTurnTick() {
                SmartDashboard.putNumber("volt", turningEncoder.getVoltage());
                SmartDashboard.putNumber("bus", RobotController.getVoltage5V());
                return turningEncoder.getVoltage() / RobotController.getVoltage5V();
            }

            public double getTurningPosition(){
                //return turningEncoder.getValue() / 36;
                //magic number made by forgetting we where multiplying by pi later so 35*pi is right
                double x = (getTurnTick() * 360) - absoluteEncoderOffsetRad;
                x = round(x, 2);
                if (x < 0) {
                    x += 360;
                }
                if(x > 180) {
                    x -= 360;
                }
                return round(x,2);
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

            public SwerveModuleState getState(){
                return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
            }
            
            private final SimpleMotorFeedforward m_turnFeed = new SimpleMotorFeedforward(1, 0.5);
            public void setDesiredState(SwerveModuleState state) {
                if (Math.abs(state.speedMetersPerSecond) < 0.01) {
                    stop();
                    return;
                }

                state = SwerveModuleState.optimize(state, getState().angle);
                driveMotor.set(round(state.speedMetersPerSecond, 3) / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
                //set turning motor mode to positional change 
                //this is the part that is not working most likely 
                //SmartDashboard.putNumber("turning position", getTurningPosition());
                final double turnFeed = m_turnFeed.calculate(turningPidController.getSetpoint().velocity);
                final double turnOut = turningPidController.calculate((getTurningPosition())*((2*Math.PI)/360), state.angle.getRadians());
                //if (!(getTurningPosition() >= state.angle.getDegrees() - 3 && getTurningPosition() <= state.angle.getDegrees() + 3)) {
                turningMotor.setVoltage(turnOut + turnFeed);
                //}
                SmartDashboard.putNumber("turn out", turnOut);
                SmartDashboard.putNumber("turn feed", turnFeed);
                SmartDashboard.putNumber("turning speed input", turnOut + turnFeed);
                //turningMotor.set(ControlMode.PercentOutput, 0.3);
                //turningMotor.set(VictorSPXControlMode.PercentOutput, state.angle.getRadians());
                
            }

            public double getRawEncoder() {
                return turningEncoder.getValue();
            }

            public static double round(double num, int dec) {
                dec = (int)Math.pow(10, dec);
                num *= 100;
                num = Math.ceil(num);
                num /= 100;
                return num;
            }

            public void stop() {
                driveMotor.set(0);
                resetEncoders();
                turningMotor.set(VictorSPXControlMode.Disabled, 0);
            }
}
