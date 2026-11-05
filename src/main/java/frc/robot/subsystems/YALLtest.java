
package frc.robot.subsystems;

import java.util.Optional;

import limelight.Limelight;
import limelight.sim.LimelightSim;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.PoseEstimate;
import limelight.networktables.Orientation3d;
import limelight.networktables.AngularVelocity3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import static edu.wpi.first.units.Units.DegreesPerSecond;

public class YALLtest {

    private final Limelight limelight =
        new Limelight("TURRET_LIMELIGHT");

    private final LimelightSim limelightSim =
        new LimelightSim(limelight);

    // Camera offset relative to the robot center.
    // Replace these zero values with actual measurements.
    private final Pose3d cameraOffset = new Pose3d(
    0.012522,   // X: Forward
   -0.276809,   // Y: Right (negative Y)
    0.438429,   // Z: Up
    Rotation3d.kZero
);

    // YALL MegaTag2 vision estimator.
    private final LimelightPoseEstimator visionPoseEstimator;

    public YALLtest() {

        // Configure the real Limelight.
        limelight.getSettings()
            .withLimelightLEDMode(LEDMode.PipelineControl)
            .withCameraOffset(cameraOffset)
            .save();

        // Configure the simulated Limelight.
        limelightSim.withRobotToCameraTransform(
            new Transform3d(
                cameraOffset.getTranslation(),
                cameraOffset.getRotation()
            )
        );

        // Initialize MegaTag2.
        visionPoseEstimator =
            limelight.createPoseEstimator(EstimationMode.MEGATAG2);
    }

    // Send current robot orientation to Limelight.
    // Call this before requesting a MegaTag2 estimate.
    public void updateRobotOrientation(Rotation3d robotRotation) {

        limelight.getSettings()
            .withRobotOrientation(
                new Orientation3d(
                    robotRotation,
                    new AngularVelocity3d(
                        DegreesPerSecond.of(0),
                        DegreesPerSecond.of(0),
                        DegreesPerSecond.of(0)
                    )
                )
            )
            .save();
    }

    // Retrieve a filtered MegaTag2 pose estimate.
    public Optional<PoseEstimate> getVisionEstimate() {

        Optional<PoseEstimate> visionEstimate =
            visionPoseEstimator.getPoseEstimate();

        return visionEstimate.filter(pose ->
            pose.hasData
            && pose.isMegaTag2
            && pose.avgTagDist < 4
            && pose.tagCount > 0
            && pose.getMinTagAmbiguity() < 0.3
        );
    }

    // Existing target detection functionality.
    public void checkTargets() {

        if (limelight.getData().targetData.getTargetStatus()) {

            double tx =
                limelight.getData().targetData.getHorizontalOffset();

            double ty =
                limelight.getData().targetData.getVerticalOffset();

            // Turn toward tx, adjust range using ty.
        }
    }

    // Simulation update.
    public void updateSimulation(Pose2d simulatedRobotPose) {

        // Supply simulated robot heading to MegaTag2.
        updateRobotOrientation(
            new Rotation3d(
                0,
                0,
                simulatedRobotPose.getRotation().getRadians()
            )
        );

        limelightSim.update(simulatedRobotPose);
    }
}
