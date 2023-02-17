package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;

public class EndEffector {
    //clockwise for cube counter clockwise for cone
    private static CANSparkMax endMotor = new CANSparkMax(DriveConstants.kEndCANId, MotorType.kBrushless);

    public static void cubeIntake(double speed) {
        endMotor.set(speed);
    }

    public static void coneIntake(double speed) {
        endMotor.set(-speed);
    }
    
    public static void cubeOut() {
        endMotor.set(-0.5);
    }

    public static void coneOut() {
        endMotor.set(0.5);
    }

    public static void stop() {
        endMotor.set(0);
    }
}
