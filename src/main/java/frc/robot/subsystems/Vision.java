// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.OptionalDouble;
import java.util.stream.IntStream;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

/**
 * Reads the Limelight once per loop and answers two questions about the HUB:
 * "which way is it?" ({@link #getTxDegrees()}) and "how far is it?" ({@link #getDistanceMeters()}).
 * Requires the Limelight to be running an AprilTag pipeline.
 */
public class Vision extends SubsystemBase {

  /** Both alliances' HUB tags, used only until the Driver Station reports our alliance. */
  private static final int[] ALL_HUB_TAG_IDS =
      IntStream.concat(
              IntStream.of(LimelightConstants.HUB_TAG_IDS_RED),
              IntStream.of(LimelightConstants.HUB_TAG_IDS_BLUE))
          .toArray();

  private final String m_limelight = LimelightConstants.TURRET_LIMELIGHT_NAME;

  private final MedianFilter m_distanceFilter = new MedianFilter(5);

  private boolean m_hasTarget = false;
  private int m_tagId = -1;
  private double m_txErrorDeg = 0.0;

  private double m_distanceMeters = Double.NaN;
  private double m_lastDistanceTimestamp = Double.NEGATIVE_INFINITY;

  @Override
  public void periodic() {
    updateFromLimelight();

    SmartDashboard.putBoolean("Vision Has Target", m_hasTarget);
    SmartDashboard.putNumber("Vision Tag ID", m_tagId);
    SmartDashboard.putNumber("Vision Tx Error (deg)", m_txErrorDeg);
    SmartDashboard.putBoolean("Vision Aimed", isAimed());
    SmartDashboard.putNumber("Vision Distance (m)", getDistanceMeters().orElse(-1.0));
  }

  /** True if a HUB tag for our alliance is visible in the latest frame. */
  public boolean hasTarget() {
    return m_hasTarget;
  }

  /** Horizontal error in degrees. Positive = HUB is to the right. Only valid while hasTarget(). */
  public double getTxDegrees() {
    return m_txErrorDeg;
  }

  /** True if a tag is visible and the robot is within AIM_TOLERANCE_DEG of centered on it. */
  public boolean isAimed() {
    return m_hasTarget && Math.abs(m_txErrorDeg) <= LimelightConstants.AIM_TOLERANCE_DEG;
  }

  /** Filtered distance to the HUB tags in meters; empty if no tag seen within TARGET_HOLD_SECONDS. */
  public OptionalDouble getDistanceMeters() {
    boolean fresh =
        Timer.getFPGATimestamp() - m_lastDistanceTimestamp <= LimelightConstants.TARGET_HOLD_SECONDS;
    return (fresh && !Double.isNaN(m_distanceMeters))
        ? OptionalDouble.of(m_distanceMeters)
        : OptionalDouble.empty();
  }

  // ---------------------------------------------------------------------------------------------

  private void updateFromLimelight() {
    // Standard Limelight outputs (always published): tv = target valid, tid = primary tag ID,
    // txnc/tync = angles from the camera's center.
    boolean valid = LimelightHelpers.getTV(m_limelight);
    int id = (int) LimelightHelpers.getFiducialID(m_limelight);
    double tx = LimelightHelpers.getTXNC(m_limelight);
    double ty = LimelightHelpers.getTYNC(m_limelight);
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
    m_txErrorDeg = found ? tx - LimelightConstants.AIM_OFFSET_DEG : 0.0;

    if (found) {
      double distance = distanceFromTy(ty);
      if (!Double.isNaN(distance)) {
        m_distanceMeters = m_distanceFilter.calculate(distance);
        m_lastDistanceTimestamp = Timer.getFPGATimestamp();
      }
    } else if (getDistanceMeters().isEmpty()) {
      m_distanceFilter.reset();
      m_distanceMeters = Double.NaN;
    }
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

  /** Our alliance's HUB tags; if the alliance is not known yet, accept every HUB tag. */
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