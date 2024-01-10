package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.VisionConstants.Node;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;

/**
 * Encapsulated PhotonCamera object used in posed estimation and alignment
 * 
 * @author Rohin Sood
 */
public class Limelight extends SubsystemBase {

    private final String limelightName;

    private final PhotonCamera camera = new PhotonCamera("limelight");
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    

    private final NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight");

    private final NetworkTableEntry ledMode = limeTable.getEntry("ledMode");

    public enum VisionLEDMode{
        kOn,
        kOff,
        kBlink
    }

    // private final ShuffleData<Boolean> targetFound = new ShuffleData<Boolean>("Limelight", "Target Found", false);
    
    // private final ShuffleData<Double> targetPitch = new ShuffleData<Double>("Limelight", "Target Pitch", 0.0);
    // private final ShuffleData<Double> targetYaw = new ShuffleData<Double>("Limelight", "Target Yaw", 0.0);
    private final ShuffleData<Integer> pipeline = new ShuffleData<Integer>("Limelight",
            "Pipeline", -1000);

    // private final ShuffleData<Integer> aprilTagID = new ShuffleData<Integer>("Limelight", "Fiducial ID", -1000);
    // private final ShuffleData<Double> aprilTagX = new ShuffleData<Double>("Limelight", "AprilTag Y", -1000.0);
    // private final ShuffleData<Double> aprilTagY = new ShuffleData<Double>("Limelight", "AprilTag X", -1000.0);

    // private final ShuffleData<Double> targetTransX = new ShuffleData<Double>("Limelight", "Target Trans X", -1000.0);
    // private final ShuffleData<Double> targetTransY = new ShuffleData<Double>("Limelight", "Target Trans Y", -1000.0);

    public Limelight(String name) {
        limelightName = name;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    camera, Constants.VisionConstants.robot_to_cam);
        } catch (Exception e) {
            System.out.println(e);
        }

        setLED(VisionLEDMode.kOff);
    }

    public LimelightHelpers.LimelightResults getLatestResult() {
        return LimelightHelpers.getLatestResults(limelightName);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public LimelightHelpers.Results getTargets(LimelightHelpers.LimelightResults result) {
        return result.targetingResults;
    }

    public LimelightTarget_Fiducial getBestTarget(LimelightHelpers.LimelightTarget_Fiducial[] result) {
        if (!hasTarget()){
            return null;
        }
        double maxTA = 0;
        LimelightTarget_Fiducial optimal = result[0];
        for (LimelightTarget_Fiducial target : result){
            if (target.ta > maxTA){
                optimal = target;
            }
        }
        return optimal;
    }

    public Rotation2d getYaw(LimelightHelpers.Results target, int index) {
        return new Rotation2d(Math.toRadians(target.botpose[index]));
    }

    public double getPitch(LimelightTarget_Fiducial target) {
        return target.tx;
    }

    public double getArea(LimelightTarget_Fiducial target) {
        return target.ta;
    }

    public double getSkew(LimelightTarget_Fiducial target) {
        return target.ty;
    }

    public double getPoseAbmiguity(AprilTagPoseEstimate tagPose) {
        
        return tagPose.getAmbiguity();
    }
    public static double calculateDistanceToTargetMeters(
        double cameraHeightMeters,
        double targetHeightMeters,
        double cameraPitchRadians,
        double targetPitchRadians) {
        return (targetHeightMeters - cameraHeightMeters)
                / Math.tan(cameraPitchRadians + targetPitchRadians);
    }


    public double getDistance(LimelightTarget_Fiducial target, Node node) {
        return calculateDistanceToTargetMeters(
                Constants.VisionConstants.camera_height,
                node.height_meters,
                Constants.VisionConstants.camera_pitch,
                Units.degreesToRadians(getPitch(target)));
    }
    public static Translation2d estimateCameraToTargetTranslation(
        double targetDistanceMeters, Rotation2d yaw) {
        return new Translation2d(
                yaw.getCos() * targetDistanceMeters, yaw.getSin() * targetDistanceMeters);
    }

    public Translation2d getTranslation2d(LimelightTarget_Fiducial target, Node node) {
        return estimateCameraToTargetTranslation(
                getDistance(target, node), new Rotation2d(target.ts));
    }

    public AprilTagFieldLayout getAprilTagFieldLayout() {
        return aprilTagFieldLayout;
    }

    public int getPipeline() {
        return camera.getPipelineIndex();
    }

    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    public void setLED(VisionLEDMode ledMode) {
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        switch (ledMode) {
            case kOn:
                LimelightHelpers.setLEDMode_ForceOn(limelightName);
                break;
            case kOff:
                LimelightHelpers.setLEDMode_ForceOff(limelightName);
                break;
            case kBlink:
                LimelightHelpers.setLEDMode_ForceBlink(limelightName);(limelightName);
                break;
            default:
                LimelightHelpers.setLEDMode_ForceOn(limelightName);
                break;
        }
    }

    public void updatePoseAprilTags(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
        LimelightHelpers.LimelightResults result = getLatestResult();
        var filter = result.getTargets().stream()
                .filter(t -> t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                .findFirst();
        if (filter.isPresent()) {
            Double imageCaptureTime = result.getTimestampSeconds();
            Optional<EstimatedRobotPose> poseEstimate = getEstimatedGlobalPose(
                    swerveDrivePoseEstimator.getEstimatedPosition());
            // if it recieved a pose update
            if (poseEstimate.isPresent()) {
                Pose2d newPose = poseEstimate.get().estimatedPose.toPose2d();

                if (DriverStation.getAlliance().equals(Alliance.Red) && DriverStation.isAutonomous())
                    ;
                // if the update is sufficiantly different to the current one, done to not cause
                // pid oscilation
                if (!Constants.withinMargin(0.04, newPose.getTranslation(),
                        swerveDrivePoseEstimator.getEstimatedPosition().getTranslation())) {
                    swerveDrivePoseEstimator.addVisionMeasurement(
                            newPose, imageCaptureTime);
                }
                // swerveDrivePoseEstimator.setVisionMeasurementStdDevs(null);
            }
        }
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
    
    public void logging() {
        
        pipeline.set(getPipeline());

    }

    @Override
    public void periodic() {

        logging();
    }

}