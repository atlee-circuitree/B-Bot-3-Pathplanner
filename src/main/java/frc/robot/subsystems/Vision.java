// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.OptionalDouble;
import java.util.stream.IntStream;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

/**
 * Reads the Limelight once per loop: "which way is the HUB?" and "how far is it?".
 * Requires an AprilTag pipeline that only tracks the CENTERED HUB tags.
 * Side-aware: the HUB opening is ~0.6 m behind the tag face, so from the side the aim point is
 * shifted toward straight-ahead; the correction uses the camera's position relative to the tag.
 */
public class Vision extends SubsystemBase {

  private static final int[] ALL_HUB_TAG_IDS =
      IntStream.concat(
              IntStream.of(LimelightConstants.HUB_TAG_IDS_RED),
              IntStream.of(LimelightConstants.HUB_TAG_IDS_BLUE))
          .toArray();

  private static final double MIN_TAG_DEPTH_METERS = 0.3;

  private final String m_limelight = LimelightConstants.TURRET_LIMELIGHT_NAME;

  private final MedianFilter m_distanceFilter = new MedianFilter(5);
  private final MedianFilter m_sideFilter = new MedianFilter(5);

  private boolean m_hasTarget = false;
  private int m_tagId = -1;
  private double m_txErrorDeg = 0.0;
  private double m_sideCorrectionDeg = 0.0;

  private double m_distanceMeters = Double.NaN;
  private double m_lastDistanceTimestamp = Double.NEGATIVE_INFINITY;

  @Override
  public void periodic() {
    updateFromLimelight();

    SmartDashboard.putBoolean("Vision Has Target", m_hasTarget);
    SmartDashboard.putNumber("Vision Tag ID", m_tagId);
    SmartDashboard.putNumber("Vision Tx Error (deg)", m_txErrorDeg);
    SmartDashboard.putNumber("Vision Side Correction (deg)", m_sideCorrectionDeg);
    SmartDashboard.putBoolean("Vision Aimed", isAimed());
    SmartDashboard.putNumber("Vision Distance (m)", getDistanceMeters().orElse(-1.0));
  }

  public boolean hasTarget() {
    return m_hasTarget;
  }

  /** Horizontal aim error in degrees. Positive = HUB is to the right. */
  public double getTxDegrees() {
    return m_txErrorDeg;
  }

  public boolean isAimed() {
    return m_hasTarget && Math.abs(m_txErrorDeg) <= LimelightConstants.AIM_TOLERANCE_DEG;
  }

  public OptionalDouble getDistanceMeters() {
    boolean fresh =
        Timer.getFPGATimestamp() - m_lastDistanceTimestamp <= LimelightConstants.TARGET_HOLD_SECONDS;
    return (fresh && !Double.isNaN(m_distanceMeters))
        ? OptionalDouble.of(m_distanceMeters)
        : OptionalDouble.empty();
  }

  // ---------------------------------------------------------------------------------------------

  private void updateFromLimelight() {
    boolean valid = LimelightHelpers.getTV(m_limelight);
    int id = (int) LimelightHelpers.getFiducialID(m_limelight);
    double tx = LimelightHelpers.getTXNC(m_limelight);
    double ty = LimelightHelpers.getTYNC(m_limelight);

    // Camera is mounted upside down: its left-right and up-down angles come out reversed.
    if (LimelightConstants.CAMERA_FLIP_TX) {
      tx = -tx;
    }
    if (LimelightConstants.CAMERA_FLIP_TY) {
      ty = -ty;
    }

    SmartDashboard.putBoolean("Vision LL tv", valid);
    SmartDashboard.putNumber("Vision LL tid", id);

    boolean found = valid && contains(hubTagIdsForAlliance(), id);

    m_hasTarget = found;
    m_tagId = found ? id : -1;

    if (found) {
      m_sideCorrectionDeg = m_sideFilter.calculate(hubCenterCorrectionDeg());
      m_txErrorDeg = tx - LimelightConstants.AIM_OFFSET_DEG - m_sideCorrectionDeg;

      double distance = distanceFromTy(ty);
      if (!Double.isNaN(distance)) {
        m_distanceMeters = m_distanceFilter.calculate(distance);
        m_lastDistanceTimestamp = Timer.getFPGATimestamp();
      }
    } else {
      m_txErrorDeg = 0.0;
      m_sideCorrectionDeg = 0.0;
      m_sideFilter.reset();
      if (getDistanceMeters().isEmpty()) {
        m_distanceFilter.reset();
        m_distanceMeters = Double.NaN;
      }
    }
  }

  /**
   * Degrees to shift the aim point so it lands on the HUB center instead of on the tag.
   * Uses camerapose_targetspace [x, y, z, roll, pitch, yaw]: x = sideways along the face,
   * z = distance out from the face. Correction = atan(lat/depth) - atan(lat/(depth+0.604)),
   * signed by which side of the tag's center line the camera is on.
   */
  private double hubCenterCorrectionDeg() {
    if (!LimelightConstants.USE_SIDE_CORRECTION) {
      return 0.0;
    }
    double[] pose = LimelightHelpers.getLimelightNTDoubleArray(m_limelight, "camerapose_targetspace");
    if (pose.length < 3) {
      return 0.0;
    }
    double lateral = pose[0];
    double depth = Math.abs(pose[2]);

    SmartDashboard.putNumber("Vision Cam Lateral (m)", lateral);
    SmartDashboard.putNumber("Vision Cam Depth (m)", depth);

    if (Double.isNaN(lateral) || Double.isNaN(depth) || depth < MIN_TAG_DEPTH_METERS) {
      return 0.0;
    }

    double sideAngle = Math.atan2(Math.abs(lateral), depth);
    double centerAngle =
        Math.atan2(Math.abs(lateral), depth + LimelightConstants.HUB_CENTER_DEPTH_METERS);
    double shiftDeg = Math.toDegrees(sideAngle - centerAngle);

    double signed = -Math.signum(lateral) * LimelightConstants.SIDE_CORRECTION_SIGN * shiftDeg;
    return MathUtil.clamp(
        signed,
        -LimelightConstants.SIDE_CORRECTION_MAX_DEG,
        LimelightConstants.SIDE_CORRECTION_MAX_DEG);
  }

  /** d = (tagHeight - lensHeight) / tan(cameraPitch + ty). NaN if the angle is not 0 to 90 deg. */
  private static double distanceFromTy(double tyDegrees) {
    double angleDeg = LimelightConstants.CAMERA_PITCH_DEGREES + tyDegrees;
    if (angleDeg <= 0.0 || angleDeg >= 90.0) {
      return Double.NaN;
    }
    double heightDiff =
        LimelightConstants.HUB_TAG_HEIGHT_METERS - LimelightConstants.CAMERA_LENS_HEIGHT_METERS;
    return heightDiff / Math.tan(Math.toRadians(angleDeg));
  }

  private static int[] hubTagIdsForAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Red
          ? LimelightConstants.HUB_TAG_IDS_RED
          : LimelightConstants.HUB_TAG_IDS_BLUE;
    }
    return ALL_HUB_TAG_IDS;
  }

  private static boolean contains(int[] ids, int id) {
    for (int candidate : ids) {
      if (candidate == id) {
        return true;
      }
    }
    return false;
  }
}