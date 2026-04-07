package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    public VisionSubsystem() {
        // No need to manually get NetworkTable, LimelightHelpers handles it
    }

    // ==================== BASIC TARGET DATA ====================

    /** Returns true if a valid target is detected */
    public boolean hasTarget() {
        return LimelightHelpers.getTV("limelight");
    }

    /** Horizontal angle to target in degrees (-27 to 27) */
    public double getTargetX() {
        return LimelightHelpers.getLimelightNTDouble("limelight", "tx");
    }

    /** Vertical angle to target in degrees (-20.5 to 20.5) */
    public double getTargetY() {
        return LimelightHelpers.getLimelightNTDouble("limelight", "ty");
    }

    /** Target area (0% to 100% of image) */
    public double getTargetArea() {
        return LimelightHelpers.getLimelightNTDouble("limelight", "ta");
    }

    /** AprilTag ID (-1 if none) */
    public int getAprilTagID() {
        return (int) LimelightHelpers.getLimelightNTDouble("limelight", "tid");
    }

    // ==================== AI OBJECT DETECTION (HAILO-8) ====================

    /** Returns the class name of detected object (e.g., "coral", "algae") */
    public String getDetectedClass() {
        return LimelightHelpers.getLimelightNTString("limelight", "tclass");
    }

    /** Returns true if a specific game piece is detected */
    public boolean isGamePieceDetected(String className) {
        return hasTarget() && getDetectedClass().equals(className);
    }

    // ==================== ROBOT LOCALIZATION ====================

    /** Returns robot pose from AprilTag detection */
    public Pose2d getRobotPose() {
        Pose2d pose = LimelightHelpers.getBotPose2d("limelight");
        double[] botpose = {pose.getX(), pose.getY(), 0, 0, 0, pose.getRotation().getDegrees()};
        return new Pose2d(
            botpose[0],  // x (meters)
            botpose[1],  // y (meters)
            Rotation2d.fromDegrees(botpose[5])  // yaw (degrees)
        );
    }

    // ==================== PIPELINE CONTROL ====================

    /** Switch between vision pipelines (0-9) */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex("limelight", pipeline);
    }

    /** Set LED mode: 0=pipeline, 1=off, 2=blink, 3=on */
    public void setLEDMode(int mode) {
        if (mode ==0){
            LimelightHelpers.setLEDMode_PipelineControl("limelight");
        }
        else if (mode ==1){
            LimelightHelpers.setLEDMode_ForceOff("limelight");
        }
        else if (mode ==2){
            LimelightHelpers.setLEDMode_ForceBlink("limelight");
        }
        else if (mode ==3){
            LimelightHelpers.setLEDMode_ForceOn("limelight");

        }
    }

    public double get_ta(){
        double ta = LimelightHelpers.getTA("limelight");
        return ta;
    }

    //  DISTANCE ESTIMATION
    private static final double LIMELIGHT_HEIGHT_METERS = 0.38;        // Height of Limelight lens from floor
    private static final double TARGET_HEIGHT_METERS = 1.124;           // Height of AprilTag center from floor
    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 0.0; // Degrees tilted up from horizontal

    /**
     * Calculates horizontal distance to the target using ty (vertical angle).
     * Uses: distance = (targetHeight - cameraHeight) / tan(mountAngle + ty)
     * @return distance in meters, or -1.0 if no target is detected
     */
    public double getDistanceToTarget() {
        if (!hasTarget()) return -1.0;

        double ty = getTargetY();
        double angleToTargetRadians = Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEGREES + ty);
        double distance = (TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT_METERS) / Math.tan(angleToTargetRadians);

        return distance;
    }

    // ==================== PIPELINE SHORTCUTS ====================

    /** Switch to AprilTag pipeline (pipeline 0) */
    public void useAprilTagPipeline() {
        setPipeline(VisionConstants.PIPELINE_APRILTAG);
    }

    /** Switch to Color Retroreflective pipeline (pipeline 1) */
    public void useRetroReflectivePipeline() {
        setPipeline(VisionConstants.PIPELINE_RETROREFLECTIVE);
    }

    /** Toggle between AprilTag and Retroreflective pipelines */
    public void togglePipeline() {
        double current = LimelightHelpers.getCurrentPipelineIndex("limelight");
        if (current == VisionConstants.PIPELINE_APRILTAG) {
            useRetroReflectivePipeline();
        } else {
            useAprilTagPipeline();
        }
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vision/ta", get_ta() );
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
        SmartDashboard.putNumber("Vision/TargetX", getTargetX());
        SmartDashboard.putNumber("Vision/TargetY", getTargetY());
        SmartDashboard.putNumber("Vision/DistanceMeters", getDistanceToTarget());
        SmartDashboard.putNumber("Vision/AprilTagID", getAprilTagID());
        SmartDashboard.putString("Vision/DetectedClass", getDetectedClass());
        SmartDashboard.putNumber("Vision/ActivePipeline", LimelightHelpers.getCurrentPipelineIndex("limelight"));
    }
}