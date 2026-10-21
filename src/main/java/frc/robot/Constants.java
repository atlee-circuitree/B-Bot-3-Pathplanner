// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LimelightHelpers;
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
    public static final double AutoDeploySpeed = 0.5;

    public static final double DEPLOY_MIN = 0;
    public static final double DEPLOY_MAX = 0.1;
    public static final double PRETTYDEPLOY_MAX = 98.0;
    public static final double PRETTYDEPLOY_MID = 48.0;
    public static final double BALL_POSITION = 4;
        /** +1 if a POSITIVE arm speed makes "Pivot Encoder get" go UP, -1 if it makes it go DOWN.
     *  -1 = what the old code assumed. Check on the robot (see test step A). */
    public static final double ARM_SPEED_TO_ANGLE_SIGN = -1.0;

  }

  public static class KickUp {

     public static final double KickUpSpeed = .80;



  }

  public static class Shooter {

    public static final double ShootSpeed = 1.0;
    public static final double ShootSpeedClose = 50.0;
    public static final double ShootSpeedFar = 55.0;
    public static final double ShootSpeedtest = 10.0;
    public static final double SHOOTER_THRESHOLD_RPS = 15.0;
    public static final double ShootSpeedAuto = 47.0;
    
    /**
     * Distance-to-speed table used by ShootFromDistance. Each row is
     * {distance in meters as reported by Vision "Distance (m)" on the dashboard, shooter speed in RPS}.
     * Keep rows sorted by distance; speeds between rows are linearly interpolated, and
     * distances outside the table use the nearest row.
     *
     * ASSUMPTION: these two rows are PLACEHOLDERS built from your existing Close/Far speeds.
     * Replace the distances with where Close and Far were actually tuned, then add a row for
     * every spot you test. Read the distance off the dashboard, not a tape measure.
     */
    public static final double[][] SHOT_TABLE = {
      {1.5, ShootSpeedClose},
      {2.5, 51.0},
      {3.0, ShootSpeedFar},
      {3.5, 60},
      {5.5, 63}

    };

    /** Speed used by auto-shoot until the first distance reading arrives (RPS). */
    public static final double AUTO_FALLBACK_RPS = ShootSpeedFar;
  }

  public static class CurrentLimits {
    public static final double SHOOTER_STATOR_AMPS        = 80.0;
    public static final double SHOOTER_SUPPLY_AMPS        = 40.0;
    public static final double KICKER_STATOR_AMPS         = 50.0;
    public static final double KICKER_SUPPLY_AMPS         = 40.0;
    public static final double INTAKE_WHEELS_STATOR_AMPS  = 40.0;
    public static final double INTAKE_WHEELS_SUPPLY_AMPS  = 30.0;
    public static final double INTAKE_ARM_STATOR_AMPS     = 30.0;
    public static final double INTAKE_ARM_SUPPLY_AMPS     = 20.0;
    public static final double DRIVE_STATOR_AMPS = 40.0;
    public static final double DRIVE_SUPPLY_AMPS = 40.0;
    public static final double STEER_STATOR_AMPS = 40.0;
    public static final double STEER_SUPPLY_AMPS = 30.0;
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
        /** NetworkTables name of the Limelight. Must match the name set in the Limelight web UI. */
        public static final String TURRET_LIMELIGHT_NAME = "limelight-left";   
        public static final double AIM_MIN_RATE_RAD_PER_SEC = 0.4; 
        public static final boolean CAMERA_FLIP_TX = true;
        public static final boolean CAMERA_FLIP_TY = true;
                // ---- Side-aware aiming (Vision.java) ----
        /** Aim at the HUB center, not the tag. Set false to aim straight at the tag. */
        public static final boolean USE_SIDE_CORRECTION = true;
        /** Distance from the HUB face (where the tags are) back to the HUB center, meters. */
        public static final double HUB_CENTER_DEPTH_METERS = 0.604;
        /** Set to -1.0 if shots from the sides miss in the wrong direction. */
        public static final double SIDE_CORRECTION_SIGN = 1.0;
        /** Largest side correction the code will apply, degrees. */
        public static final double SIDE_CORRECTION_MAX_DEG = 20.0;

        

        // ---- Camera mounting (used by the ty distance formula in Vision.java) ----
        // ASSUMPTION: lens height is a PLACEHOLDER (measure it). Pitch is the team's rough
        // estimate (about 15 degrees up) and will be calibrated before testing.
        /** Height of the Limelight lens above the carpet, in meters. */
        public static final double CAMERA_LENS_HEIGHT_METERS = 0.441;
        /** Upward tilt of the camera above horizontal, in degrees (0 = level, + = tilted up). */
        public static final double CAMERA_PITCH_DEGREES      = 33.5;

        // ---- Target: 2026 REBUILT HUB AprilTags ----
        /** Tag centers are 44.25 in (1.124 m) off the carpet, same for every HUB tag (2026 game manual). */
        public static final double HUB_TAG_HEIGHT_METERS = 1.124;
        // The manual lists these 16 IDs as the HUB tags but does not say which alliance owns which.
        // ASSUMPTION: Red = 2-11, Blue = 18-27. VERIFY against the field layout (see the guide).
        // Each HUB face has one centered tag and one offset tag. Once you know which IDs are the
        // centered ones, trimming these lists to only those IDs makes aiming more accurate.
         // Centered HUB tags only. Red: 10 front, 2/5 sides. Blue: 26 front, 21/18 sides.
        // (4 and 20 are the back faces toward the neutral zone; add them only if you shoot from there.)
        public static final int[] HUB_TAG_IDS_RED  = {10, 2, 5};
        public static final int[] HUB_TAG_IDS_BLUE = {26, 21, 18};

        // ---- Auto-aim (AimAtHub.java) ----
        /** Rotation command in rad/s per degree of horizontal error. Start low, raise until snappy. */
        public static final double AIM_KP                  = 0.08;
        /** Upper limit on the rotation command while aiming, in rad/s. */
        public static final double AIM_MAX_RATE_RAD_PER_SEC = 3.0;
        /** Within this many degrees of center the robot counts as "aimed" and feeding is allowed. */
        public static final double AIM_TOLERANCE_DEG       = 2.0;
        /**
         * Degrees between the camera's centerline and the direction the shooter actually points.
         * 0 if the camera looks the same way the shooter fires. Positive = shooter points to the
         * camera's right.
         */
        public static final double AIM_OFFSET_DEG          = 0.0;

        // ---- Filtering ----
        /** After the tag is lost, keep using the last distance for this long (seconds). */
        public static final double TARGET_HOLD_SECONDS     = 0.3;
    }

}
