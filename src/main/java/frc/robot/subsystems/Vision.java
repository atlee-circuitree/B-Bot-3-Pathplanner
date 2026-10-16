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
 *
 * <p>All Limelight access for aiming and ranging lives here. Commands never touch NetworkTables.
 *
 * <p><b>Distance</b> uses the fixed-angle-camera method from the Limelight docs:
 * {@code d = (targetHeight - lensHeight) / tan(cameraPitch + ty)}. It needs no robot pose, so it
 * works even though the drivetrain only fuses vision while disabled. Every HUB tag is the same
 * height, so any HUB tag gives the same answer.
 *
 * <p><b>Tag choice:</b> of the visible tags that belong to our alliance's HUB, the one closest to
 * the center of the image is used. Once the robot is rotating toward a tag, that tag stays the
 * closest-to-center one, so the choice does not flip back and forth.
 *
 * <p>Requires the Limelight to be running an AprilTag pipeline.
 */
public class Vision extends SubsystemBase {

  /** rawfiducials is a flat array: id, txnc, tync, ta, distToCamera, distToRobot, ambiguity. */
  private static final int RAW_FIDUCIAL_STRIDE = 7;

  /** Both alliances' HUB tags, used only until the Driver Station reports our alliance. */
  private static final int[] ALL_HUB_TAG_IDS =
      IntStream.concat(
              IntStream.of(LimelightConstants.HUB_TAG_IDS_RED),
              IntStream.of(LimelightConstants.HUB_TAG_IDS_BLUE))
          .toArray();

  private final String m_limelight = LimelightConstants.TURRET_LIMELIGHT_NAME;

  /** Median rejects single-frame outliers and, unlike a moving average, behaves after a reset. */
  private final MedianFilter m_distanceFilter = new MedianFilter(5);

  // Latest-frame data (valid only while m_hasTarget is true).
  private boolean m_hasTarget = false;
  private int m_tagId = -1;
  private double m_txErrorDeg = 0.0;

  // Held data (survives short dropouts so the shooter speed does not jump when a tag blinks out).
  private double m_distanceMeters = Double.NaN;
  private double m_lastDistanceTimestamp = Double.NEGATIVE_INFINITY;

  @Override
  public void periodic() {
    updateFromLimelight();

    SmartDashboard.putBoolean("Vision/Has Target", m_hasTarget);
    SmartDashboard.putNumber("Vision/Tag ID", m_tagId);
    SmartDashboard.putNumber("Vision/Tx Error (deg)", m_txErrorDeg);
    SmartDashboard.putBoolean("Vision/Aimed", isAimed());
    SmartDashboard.putNumber("Vision/Distance (m)", getDistanceMeters().orElse(-1.0));
  }

  /** True if a HUB tag for our alliance is visible in the latest frame. */
  public boolean hasTarget() {
    return m_hasTarget;
  }

  /**
   * Horizontal error to where the shooter points, in degrees. Positive means the HUB is to the
   * right, so the robot must rotate clockwise. Only meaningful while {@link #hasTarget()}.
   */
  public double getTxDegrees() {
    return m_txErrorDeg;
  }

  /** True if a tag is visible and the robot is within AIM_TOLERANCE_DEG of centered on it. */
  public boolean isAimed() {
    return m_hasTarget && Math.abs(m_txErrorDeg) <= LimelightConstants.AIM_TOLERANCE_DEG;
  }

  /**
   * Filtered horizontal distance from the camera to the HUB tags, in meters. Empty if no tag has
   * been seen within TARGET_HOLD_SECONDS.
   */
  public OptionalDouble getDistanceMeters() {
    boolean fresh =
        Timer.getFPGATimestamp() - m_lastDistanceTimestamp <= LimelightConstants.TARGET_HOLD_SECONDS;
    return (fresh && !Double.isNaN(m_distanceMeters))
        ? OptionalDouble.of(m_distanceMeters)
        : OptionalDouble.empty();
  }

  // ---------------------------------------------------------------------------------------------

  private void updateFromLimelight() {
    double[] raw = LimelightHelpers.getLimelightNTDoubleArray(m_limelight, "rawfiducials");
    int[] validIds = hubTagIdsForAlliance();

    // Pick the valid tag closest to the center of the image.
    boolean found = false;
    int bestId = -1;
    double bestTx = 0.0;
    double bestTy = 0.0;
    double bestDistToCamera = 0.0;
    for (int i = 0; i + RAW_FIDUCIAL_STRIDE <= raw.length; i += RAW_FIDUCIAL_STRIDE) {
      int id = (int) raw[i];
      if (!contains(validIds, id)) {
        continue;
      }
      double txnc = raw[i + 1];
      if (!found || Math.abs(txnc) < Math.abs(bestTx)) {
        found = true;
        bestId = id;
        bestTx = txnc;
        bestTy = raw[i + 2];
        bestDistToCamera = raw[i + 4];
      }
    }

    m_hasTarget = found;
    m_tagId = found ? bestId : -1;
    m_txErrorDeg = found ? bestTx - LimelightConstants.AIM_OFFSET_DEG : 0.0;

    if (found) {
      double distance = distanceFromTy(bestTy);
      if (!Double.isNaN(distance)) {
        m_distanceMeters = m_distanceFilter.calculate(distance);
        m_lastDistanceTimestamp = Timer.getFPGATimestamp();
      }
      // Limelight's own 3D estimate, shown only as a cross-check while calibrating pitch/height.
      SmartDashboard.putNumber("Vision/LL 3D Dist To Camera (m)", bestDistToCamera);
    } else if (getDistanceMeters().isEmpty()) {
      // Held value expired: start the filter fresh next time a tag appears.
      m_distanceFilter.reset();
      m_distanceMeters = Double.NaN;
    }
  }

  /**
   * Limelight docs, "Distance Estimation with a Fixed-Angle Camera":
   * d = (h2 - h1) / tan(a1 + a2), with ty measured from the camera's center (tync).
   *
   * @return distance in meters, or NaN if the geometry is invalid (angle not between 0 and 90 deg)
   */
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
