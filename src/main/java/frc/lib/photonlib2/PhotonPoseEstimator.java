package frc.lib.photonlib2;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * The PhotonPoseEstimator class filters or combines readings from all the AprilTags visible at a
 * given timestamp on the field to produce a single robot in field pose, using the strategy set
 * below. Example usage can be found in our apriltagExample example project.
 */
public class PhotonPoseEstimator {
    /** Position estimation strategies that can be used by the {@link PhotonPoseEstimator} class. */


    private AprilTagFieldLayout fieldTags;
    // private PoseStrategy primaryStrategy;
    // private PoseStrategy multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY;
    private final PhotonCamera camera;
    private Transform3d robotToCamera;

    private Pose3d lastPose;
    private Pose3d referencePose;
    protected double poseCacheTimestampSeconds = -1;
    private final Set<Integer> reportedErrors = new HashSet<>();

    /**
     * Create a new PhotonPoseEstimator.
     *
     * @param fieldTags A WPILib {@link AprilTagFieldLayout} linking AprilTag IDs to Pose3d objects
     *     with respect to the FIRST field using the <a
     *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system">Field
     *     Coordinate System</a>.
     * @param strategy The strategy it should use to determine the best pose.
     * @param camera PhotonCamera
     * @param robotToCamera Transform3d from the center of the robot to the camera mount position (ie,
     *     robot ➔ camera) in the <a
     *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
     *     Coordinate System</a>.
     */
    public PhotonPoseEstimator(
            AprilTagFieldLayout fieldTags,
            PhotonCamera camera,
            Transform3d robotToCamera) {
        this.fieldTags = fieldTags;
        this.camera = camera;
        this.robotToCamera = robotToCamera;
    }

    /** Invalidates the pose cache. */
    private void invalidatePoseCache() {
        poseCacheTimestampSeconds = -1;
    }

    private void checkUpdate(Object oldObj, Object newObj) {
        if (oldObj != newObj && oldObj != null && !oldObj.equals(newObj)) {
            invalidatePoseCache();
        }
    }

    /**
     * Get the AprilTagFieldLayout being used by the PositionEstimator.
     *
     * @return the AprilTagFieldLayout
     */
    public AprilTagFieldLayout getFieldTags() {
        return fieldTags;
    }

    /**
     * Set the AprilTagFieldLayout being used by the PositionEstimator.
     *
     * @param fieldTags the AprilTagFieldLayout
     */
    public void setFieldTags(AprilTagFieldLayout fieldTags) {
        checkUpdate(this.fieldTags, fieldTags);
        this.fieldTags = fieldTags;
    }


    /**
     * Return the reference position that is being used by the estimator.
     *
     * @return the referencePose
     */
    public Pose3d getReferencePose() {
        return referencePose;
    }

    /**
     * Update the stored reference pose for use when using the <b>CLOSEST_TO_REFERENCE_POSE</b>
     * strategy.
     *
     * @param referencePose the referencePose to set
     */
    public void setReferencePose(Pose3d referencePose) {
        checkUpdate(this.referencePose, referencePose);
        this.referencePose = referencePose;
    }

    /**
     * Update the stored reference pose for use when using the <b>CLOSEST_TO_REFERENCE_POSE</b>
     * strategy.
     *
     * @param referencePose the referencePose to set
     */
    public void setReferencePose(Pose2d referencePose) {
        setReferencePose(new Pose3d(referencePose));
    }

    /**
     * Update the stored last pose. Useful for setting the initial estimate when using the
     * <b>CLOSEST_TO_LAST_POSE</b> strategy.
     *
     * @param lastPose the lastPose to set
     */
    public void setLastPose(Pose3d lastPose) {
        this.lastPose = lastPose;
    }

    /**
     * Update the stored last pose. Useful for setting the initial estimate when using the
     * <b>CLOSEST_TO_LAST_POSE</b> strategy.
     *
     * @param lastPose the lastPose to set
     */
    public void setLastPose(Pose2d lastPose) {
        setLastPose(new Pose3d(lastPose));
    }

    /** @return The current transform from the center of the robot to the camera mount position */
    public Transform3d getRobotToCameraTransform() {
        return robotToCamera;
    }

    /**
     * Useful for pan and tilt mechanisms and such.
     *
     * @param robotToCamera The current transform from the center of the robot to the camera mount
     *     position
     */
    public void setRobotToCameraTransform(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
    }

    /**
     * Poll data from the configured cameras and update the estimated position of the robot. Returns
     * empty if there are no cameras set or no targets were found from the cameras.
     *
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     *     the estimate
     */
    public Optional<EstimatedRobotPose> update() {
        if (camera == null) {
            DriverStation.reportError("[PhotonPoseEstimator] Missing camera!", false);
            return Optional.empty();
        }

        PhotonPipelineResult cameraResult = camera.getLatestResult();

        return update(cameraResult);
    }

    /**
     * Updates the estimated position of the robot. Returns empty if there are no cameras set or no
     * targets were found from the cameras.
     *
     * @param cameraResult The latest pipeline result from the camera
     * @return an EstimatedRobotPose with an estimated pose, and information about the camera(s) and
     *     pipeline results used to create the estimate
     */
    public Optional<EstimatedRobotPose> update(PhotonPipelineResult cameraResult) {
        // Time in the past -- give up, since the following if expects times > 0
        if (cameraResult.getTimestampSeconds() < 0) {
            return Optional.empty();
        }

        // If the pose cache timestamp was set, and the result is from the same timestamp, return an
        // empty result
        if (poseCacheTimestampSeconds > 0
                && Math.abs(poseCacheTimestampSeconds - cameraResult.getTimestampSeconds()) < 1e-6) {
            return Optional.empty();
        }

        // Remember the timestamp of the current result used
        poseCacheTimestampSeconds = cameraResult.getTimestampSeconds();

        // If no targets seen, trivial case -- return empty result
        if (!cameraResult.hasTargets()) {
            return Optional.empty();
        }

        return multiTagPNPStrategy(cameraResult);
    }



    private Optional<EstimatedRobotPose> multiTagPNPStrategy(PhotonPipelineResult result) {
        // Arrays we need declared up front
        var visCorners = new ArrayList<TargetCorner>();
        var knownVisTags = new ArrayList<AprilTag>();
        var fieldToCams = new ArrayList<Pose3d>();
        var fieldToCamsAlt = new ArrayList<Pose3d>();

        if (result.getTargets().size() < 2) {
            // Run fallback strategy instead
            return Optional.empty();
        }

        for (var target : result.getTargets()) {
            visCorners.addAll(target.getDetectedCorners());

            var tagPoseOpt = fieldTags.getTagPose(target.getFiducialId());
            if (tagPoseOpt.isEmpty()) {
                reportFiducialPoseError(target.getFiducialId());
                continue;
            }

            var tagPose = tagPoseOpt.get();

            // actual layout poses of visible tags -- not exposed, so have to recreate
            knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));

            fieldToCams.add(tagPose.transformBy(target.getBestCameraToTarget().inverse()));
            fieldToCamsAlt.add(tagPose.transformBy(target.getAlternateCameraToTarget().inverse()));
        }

        var cameraMatrixOpt = camera.getCameraMatrix();
        var distCoeffsOpt = camera.getDistCoeffs();
        boolean hasCalibData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();

        // multi-target solvePNP
        if (hasCalibData) {
            var cameraMatrix = cameraMatrixOpt.get();
            var distCoeffs = distCoeffsOpt.get();
            var pnpResults =
                    VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, knownVisTags);
            var best =
                    new Pose3d()
                            .plus(pnpResults.best) // field-to-camera
                            .plus(robotToCamera.inverse()); // field-to-robot
            // var alt = new Pose3d()
            // .plus(pnpResults.alt) // field-to-camera
            // .plus(robotToCamera.inverse()); // field-to-robot

            return Optional.of(
                    new EstimatedRobotPose(best, result.getTimestampSeconds(), result.getTargets()));
        } else {
            // TODO fallback strategy? Should we just always do solvePNP?
            return Optional.empty();
        }
    }

    
    /**
     * Difference is defined as the vector magnitude between the two poses
     *
     * @return The absolute "difference" (>=0) between two Pose3ds.
     */
    private double calculateDifference(Pose3d x, Pose3d y) {
        return x.getTranslation().getDistance(y.getTranslation());
    }

    private void reportFiducialPoseError(int fiducialId) {
        if (!reportedErrors.contains(fiducialId)) {
            DriverStation.reportError(
                    "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + fiducialId, false);
            reportedErrors.add(fiducialId);
        }
    }
}