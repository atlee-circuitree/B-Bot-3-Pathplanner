// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.*;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class CAN_IDS {


    public static final int PivotEncoder = 15;

    public static final int IntakeArm = 9;
    public static final int IntakeWheels = 10;
    public static final int ShooterLeft = 11;
    public static final int ShooterRight = 12;
    public static final int Indexer = 13;
    public static final int KickUpMotor = 14;


  }

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;


  }

  public static class Intake {

    public static final double INTAKE_SPEED = 0.9;
    public static final double DEPLOY_SLOT0_KP = 5.0;
    public static final double DEPLOY_SLOT1_KP = 1.5;
    public static final double DeploySpeed = 0.1;
    public static final double DEPLOY_MIN = 0;
    public static final double DEPLOY_MAX = 0.1;
    public static final double PRETTYDEPLOY_MAX = 98.0;
    public static final double PRETTYDEPLOY_MID = 98.0;
    public static final double BALL_POSITION = 4;

  }

  public static class KickUp {

     public static final double KickUpSpeed = .80;



  }

  public static class Shooter {

    public static final double ShootSpeed = 1.0;
    public static final double ShootSpeedClose = 47.0;
    public static final double ShootSpeedFar = 55.0;
    public static final double ShootSpeedtest = 10.0;
    public static final double SHOOTER_THRESHOLD_RPS = 15.0;
  }

  public static class Drive {
   public static final double MAX_SPEED_MPS      = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
   /** Max angular rate (rad/s) — 0.75 rotations/sec, matches teleop drive default command. */
   public static final double MAX_ANGULAR_RATE_RPS = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

   /** Fraction of max speed/rate applied as joystick deadband. */
   public static final double DEADBAND_PERCENT   = 0.14;

   // --- Speed scaling (used in drive default command) ---
   /** Current speed divisor (1.0 = full speed). Increase for a slower mode. */
   public static final double SPEED_DIVISOR      = 1.0;


  }

  public static class LimelightConstants {
        /** NetworkTables name of the turret-facing Limelight. */
        public static final String TURRET_LIMELIGHT_NAME = "limelight-front";

        // --- Distance model:  distance = DISTANCE_SCALE * ta ^ DISTANCE_EXPONENT --- redo
        public static final double DISTANCE_SCALE      = 46.39986;
        public static final double DISTANCE_EXPONENT   = -0.4918478;
        /** ta values above this threshold are too close to use for ranging. */
        public static final double MAX_TA_FOR_DISTANCE = 24.0; //redo

        // --- Turret auto-aim thresholds (tx degrees off-center) ---
        /** Switch from fast to slow rotation below this tx error. */
        public static final double TX_THRESHOLD_LARGE  = 10.0;
        /** Stop rotating (consider locked) below this tx error. */
        public static final double TX_THRESHOLD_SMALL  = 4.5;
    }

}
