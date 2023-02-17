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

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
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


public class SwerveModuleRelative {
    
    private final CANSparkMax driveMotor;
    private final WPI_VictorSPX turningMotor;

    private final RelativeEncoder driveEncoder;
    private final Encoder turnEncoder;

    private final ProfiledPIDController turningPidController;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModuleRelative(int driveMotorID, int turningMotorId, boolean driveMotorReversed,
            boolean turningMotorReversed, int turningEncoderID, double absoluteEncoderOffset,
            boolean absoluteEncoderReversed){

                this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
                this.absoluteEncoderReversed = absoluteEncoderReversed;

                driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
                turningMotor = new WPI_VictorSPX(turningMotorId);

                turnEncoder = new Encoder(turningEncoderID, turningEncoderID+1);

                turningMotor.configFactoryDefault();
                turningMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
                turningMotor.setSelectedSensorPosition(getTurningPosition());
                //already initialized 
                //turningEncoder = new AnalogEncoder(turningEncoderID);
                driveMotor.setInverted(driveMotorReversed);
                turningMotor.setInverted(turningMotorReversed);

                driveEncoder = driveMotor.getEncoder();

                driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
                driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

                turningPidController = new ProfiledPIDController(1, 0.01, 0.01, 
                new TrapezoidProfile.Constraints(DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, 
                DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond));
                //turningPidController.enableContinuousInput(-Math.PI, Math.PI);

                resetEncoders();
            }   

            public double getDrivePosition(){
                return driveEncoder.getPosition();
            }

            public double getTurningPosition(){
                double tick = turnEncoder.getDistance() % 360;
                if (tick > 180) {
                    tick -= 360;
                }
                if (tick < -180) {
                    tick += 360;
                }

                return Math.floor(tick);
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

                //deactivated to test turning motors only
                driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

                driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

                final double turnFeed = m_turnFeed.calculate(turningPidController.getSetpoint().velocity);
                final double turnOut = turningPidController.calculate((getTurningPosition())*((2*Math.PI)/360), state.angle.getRadians());
                if(Math.abs(state.angle.getDegrees() - getTurningPosition()) >= 1 || turnOut >= 0.75) {
                    turningMotor.setVoltage(turnOut + turnFeed);
                } else {
                    turningMotor.setVoltage(0);
                }
                SmartDashboard.putNumber("turn out", turnOut);
                SmartDashboard.putNumber("turn feed", turnFeed);
                SmartDashboard.putNumber("turning speed input", turnOut + turnFeed);
                //turningMotor.set(ControlMode.PercentOutput, 0.3);
                //turningMotor.set(VictorSPXControlMode.PercentOutput, state.angle.getRadians());
                
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
