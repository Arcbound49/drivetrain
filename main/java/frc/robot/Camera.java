package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;

public class Camera {
    //320 by 240 vfov 41 hfov 54
    public static final double angleSoft = 1;
    public static final int hfov = 54;
    public static final int vfov = 41;
    public static final double CameraHeight = 0.1;
    public static final double CameraAngle = 0;
    public static final double[] TargetHeight = {1, 2, 3};
    public static final int pixelHeight = 240;
    public static final int pixelWidth = 320;
    public static double angleX;
    public static double angleY;
    public static long tv;
    public Camera(){
        //https://docs.limelightvision.io/en/latest/networktables_api.html
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0); 
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
        tv = table.getEntry("tv").getInteger(0);
        //tx and ty is offset from centre not pixel cords 
        double tx = table.getEntry("tx").getDouble(0);
        double ty = table.getEntry("ty").getDouble(0);

        nomralizedPixels(tx, ty);
    }

    public static void nomralizedPixels(double tx, double ty) {
        //nomralized between -1 and 1 
        double nx = tx / pixelWidth;
        double ny = ty / pixelHeight;
        
        double vpw = 2.0*Math.tan(hfov/2);
        double vph = 2.0*Math.tan(vfov/2);
        //normalized if needed change to public static 
        double x = vpw/2 * nx;
        double y = vph/2 * ny;
        //angle offset from centre to target point 
        angleX = Math.atan2(1, x);
        //will mainly be using y angle x angle could be used for telling robot to turn x degrees andface it so we only have to use y angle
        angleY = Math.atan2(1, y);
    }

    public static double estimateDistance(int targetLayer) {
        double distance = (TargetHeight[targetLayer] - CameraHeight) / (CameraAngle + angleY);
        return distance;
    }

    public static void softDriveTargeting() {
        //for now it will turn you to face the target
        /*
         * chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
         * SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
         */
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        if (tv == 1) {
            while (angleX > angleSoft || angleX < angleSoft) {
                chassisSpeeds.omegaRadiansPerSecond = angleSoft/10;
                SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            }
        }
    }
}