package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;

public class FourBar {
    private static CANSparkMax ArmMotor = new CANSparkMax(DriveConstants.kArmMotorCANId, MotorType.kBrushless);
    private static RelativeEncoder ArmEncoder = ArmMotor.getEncoder();
    private static PIDController armController = new PIDController(0.1, 0, 0);
    private static int currentStage = 0;

    public static void Move(int stage) {
        if (ArmEncoder.getPosition()+10 >= DriveConstants.kArmDegreeStages[3] 
        || 
        ArmMotor.getBusVoltage() > DriveConstants.kArmVoltMax
        ||
        ArmMotor.getOutputCurrent() > DriveConstants.kArmCurMax) {
            return;
        }
        switch(stage) {
            case 1:
            ArmMotor.set(armController.calculate(ArmEncoder.getPosition(), DriveConstants.kArmDegreeStages[0]));
            setStage(1);
            break;

            case 2:
            ArmMotor.set(armController.calculate(ArmEncoder.getPosition(), DriveConstants.kArmDegreeStages[1]));
            setStage(2);
            break;

            case 3:
            ArmMotor.set(armController.calculate(ArmEncoder.getPosition(), DriveConstants.kArmDegreeStages[2]));
            setStage(3);
            break;


            default:
            ArmMotor.set(armController.calculate(ArmEncoder.getPosition(), 0));
            setStage(0);
            break;

            case 0:
            ArmMotor.set(armController.calculate(ArmEncoder.getPosition(), 0));
            setStage(0);
            break;
        }
    }

    public static int getStage() {
        return currentStage;
    }

    private static void setStage(int set) {
        currentStage = set;
    }
}
