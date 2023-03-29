package frc.lib.photonlib2;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.estimation.PNPResults;
import org.photonvision.estimation.RotTrlTransform3d;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.TargetCorner;

public class VisionEstimation {
    public static final TargetModel kTagModel =
            new TargetModel(Units.inchesToMeters(6), Units.inchesToMeters(6));

    /**
     * Performs solvePNP using 3d-2d point correspondences to estimate the field-to-camera
     * transformation. If only one tag is visible, the result may have an alternate solution.
     *
     * <p><b>Note:</b> The returned transformation is from the field origin to the camera pose!
     *
     * @param cameraMatrix the camera intrinsics matrix in standard opencv form
     * @param distCoeffs the camera distortion matrix in standard opencv form
     * @param corners The visible tag corners in the 2d image
     * @param knownTags The known tag field poses corresponding to the visible tag IDs
     * @return The transformation that maps the field origin to the camera pose
     */
    public static PNPResults estimateCamPosePNP(
            Matrix<N3, N3> cameraMatrix,
            Matrix<N5, N1> distCoeffs,
            List<TargetCorner> corners,
            List<AprilTag> knownTags) {
        if (knownTags == null
                || corners == null
                || corners.size() != knownTags.size() * 4
                || knownTags.size() == 0) {
            return new PNPResults();
        }
        
        // multi-tag pnp
        if (corners.size() >= 8){
            var objectTrls = new ArrayList<Translation3d>();
            for (var tag : knownTags) objectTrls.addAll(kTagModel.getFieldVertices(tag.pose));
            var camToOrigin = OpenCVHelp.solvePNP_SQPNP(cameraMatrix, distCoeffs, objectTrls, corners);
            // var camToOrigin = OpenCVHelp.solveTagsPNPRansac(prop, objectTrls, corners);
            return new PNPResults(
                    camToOrigin.best.inverse(),
                    camToOrigin.alt.inverse(),
                    camToOrigin.ambiguity,
                    camToOrigin.bestReprojErr,
                    camToOrigin.altReprojErr);
        }
        else{
            return new PNPResults();
        }
    }

    
    /**
     * The best estimated transformation (Rotation-translation composition) that maps a set of
     * translations onto another with point correspondences, and its RMSE.
     */
    public static class SVDResults {
        public final RotTrlTransform3d trf;
        /** If the result is invalid, this value is -1 */
        public final double rmse;

        public SVDResults() {
            this(new RotTrlTransform3d(), -1);
        }

        public SVDResults(RotTrlTransform3d trf, double rmse) {
            this.trf = trf;
            this.rmse = rmse;
        }
    }
}