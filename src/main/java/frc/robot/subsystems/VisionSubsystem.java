package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;

/**
 * ! hullo
 * 
 * I noticed that you're using Networktables to directly
 * control limelights. You should probably use the LimelightHelper class instead lmao
 * 
 * I added some basic stuff for megatag localization below;
 * you can add more stuff if you want to use limelights other features
 * 
 * ! NOTE THAT YOU STILL NEED TO CONFIGURE YO LIMELIGHTS AND STUFF (use docs)
 * @see https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary
 */
public class VisionSubsystem extends SubsystemBase {
    /**
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
     */
    public static enum MegatagMode {
        MEGATAG1,
        MEGATAG2
    }

    private MegatagMode mMegatagMode = MegatagMode.MEGATAG1;

    public VisionSubsystem() {
        /** Init each camera for megatag localization */
        for (var cam : Constants.kCameras) {
            var displacement = cam.displacementWrtChassis();        

            LimelightHelpers.setCameraPose_RobotSpace(
                cam.name(), 
                displacement.getX(), 
                displacement.getY(), 
                displacement.getZ(), 
                displacement.getRotation().getX(), 
                displacement.getRotation().getY(), 
                displacement.getRotation().getZ());
        }
    }

    /**
     * Set megatag localization mode
     * 
     * @param megatagMode {@link MegatagMode}
     */
    public void setMegatagMode(MegatagMode megatagMode) {
        this.mMegatagMode = megatagMode;
    }

    /**
     * Get position estimates from all limelights
     * 
     * @param gyroOrientation {@link Rotation2d} of current robot
     * @return {@link PoseEstimate} {@link Array}
     */
    public PoseEstimate[] getPoseEstimates(Rotation2d gyroOrientation) {
        ArrayList<PoseEstimate> estimates = new ArrayList<>();

        // For each camera
        for (var cam : Constants.kCameras) {
            // Set cam orientation to chassis yaw + alliance modifier
            LimelightHelpers.SetRobotOrientation(
                cam.name(), 
                gyroOrientation.plus(Constants.getAlliance() == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg).getDegrees(),
                0.0, 
                0.0,
                0.0, 
                0.0, 
                0.0);

            // Get megatag estimate
            var estimate = (mMegatagMode == MegatagMode.MEGATAG1)
                ? getMegatag1PoseEstimate(cam.name())
                : getMegatag2PoseEstimate(cam.name());

            // Reject null/no tag estimates
            if (estimate != null && estimate.tagCount > 0) {
                estimates.add(estimate);

                SmartDashboard.putBoolean("Vision/" + cam.name() + "/Has Data", true);
            } else SmartDashboard.putBoolean("Vision/" + cam.name() + "/Has Data", true);
        }

        // Cast and return arraylist
        return estimates.toArray(new PoseEstimate[] {});
    }

    /**
     * Get megatag1 position estimate for a camera
     * 
     * @param cameraName {@link String} representing camera name
     * @return {@link PoseEstimate}
     */
    private PoseEstimate getMegatag1PoseEstimate(String cameraName) {
        return Constants.getAlliance() == Alliance.Blue
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName)
            : LimelightHelpers.getBotPoseEstimate_wpiRed(cameraName);
    }

    /**
     * Get megatag2 position estimate for a camera
     * 
     * @param cameraName {@link String} representing camera name
     * @return {@link PoseEstimate}
     */
    private PoseEstimate getMegatag2PoseEstimate(String cameraName) {
        return Constants.getAlliance() == Alliance.Blue
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName)
            : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(cameraName);
    }

    // ! Why download the LimelightHelpers class to your codebase if you aren't gonna use it :(
    // ! I'll leave this here if you want but like yeah :C
    // ! :<
    // // ==================== BASIC TARGET DATA ====================

    // /** Returns true if a valid target is detected */
    // public boolean hasTarget() {
    //     return limelight.getEntry("tv").getDouble(0) == 1;
    // }

    // /** Horizontal angle to target in degrees (-27 to 27) */
    // public double getTargetX() {
    //     return limelight.getEntry("tx").getDouble(0);
    // }

    // /** Vertical angle to target in degrees (-20.5 to 20.5) */
    // public double getTargetY() {
    //     return limelight.getEntry("ty").getDouble(0);
    // }

    // /** Target area (0% to 100% of image) */
    // public double getTargetArea() {
    //     return limelight.getEntry("ta").getDouble(0);
    // }

    // /** AprilTag ID (-1 if none) */
    // public int getAprilTagID() {
    //     return (int) limelight.getEntry("tid").getDouble(-1);
    // }

    // // ==================== AI OBJECT DETECTION (HAILO-8) ====================

    // /** Returns the class name of detected object (e.g., "coral", "algae") */
    // public String getDetectedClass() {
    //     return limelight.getEntry("tclass").getString("none");
    // }

    // /** Returns true if a specific game piece is detected */
    // public boolean isGamePieceDetected(String className) {
    //     return hasTarget() && getDetectedClass().equals(className);
    // }

    // // ==================== ROBOT LOCALIZATION ====================

    // /** Returns robot pose from AprilTag detection */
    // public Pose2d getRobotPose() {
    //     double[] botpose = limelight.getEntry("botpose").getDoubleArray(new double[6]);
    //     return new Pose2d(
    //         botpose[0],  // x (meters)
    //         botpose[1],  // y (meters)
    //         Rotation2d.fromDegrees(botpose[5])  // yaw (degrees)
    //     );
    // }

    // // ==================== PIPELINE CONTROL ====================

    // /** Switch between vision pipelines (0-9) */
    // public void setPipeline(int pipeline) {
    //     limelight.getEntry("pipeline").setNumber(pipeline);
    // }

    // /** Set LED mode: 0=pipeline, 1=off, 2=blink, 3=on */
    // public void setLEDMode(int mode) {
    //     limelight.getEntry("ledMode").setNumber(mode);
    // }

    // // ==================== PERIODIC ====================

    // @Override
    // public void periodic() {
    //     SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
    //     SmartDashboard.putNumber("Vision/TargetX", getTargetX());
    //     SmartDashboard.putNumber("Vision/TargetY", getTargetY());
    //     SmartDashboard.putNumber("Vision/AprilTagID", getAprilTagID());
    //     SmartDashboard.putString("Vision/DetectedClass", getDetectedClass());
    // }
}