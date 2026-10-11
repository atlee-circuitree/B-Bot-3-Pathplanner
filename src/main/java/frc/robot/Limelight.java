package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Limelight {
    private static final NetworkTable limelight = NetworkTableInstance.getDefault()
            .getTable(Constants.LimelightConstants.TURRET_LIMELIGHT_NAME);

    public static double getTA() {
        return limelight.getEntry("ta").getDouble(0.0);
    }

    public static double getTA(String limelightName) {
        return NetworkTableInstance.getDefault()
            .getTable(limelightName).getEntry("ta").getDouble(0.0);
    }

    public static double getTX() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public static double getTY() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    //magic number 46.39986 no documentation
    public static double getDistance() {
        double ta = getTA();
        if (ta > Constants.LimelightConstants.MAX_TA_FOR_DISTANCE || ta == 0) {
            return 0;
        }
        return Constants.LimelightConstants.DISTANCE_SCALE
                * Math.pow(ta, Constants.LimelightConstants.DISTANCE_EXPONENT);
    }

    public static double getDistance(String limelightName) {
        double ta = getTA(limelightName);
        if (ta > Constants.LimelightConstants.MAX_TA_FOR_DISTANCE || ta == 0) {
            return 0;
        }
        return Constants.LimelightConstants.DISTANCE_SCALE
                * Math.pow(ta, Constants.LimelightConstants.DISTANCE_EXPONENT);
    }
}
