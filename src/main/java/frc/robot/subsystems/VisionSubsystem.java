package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable limelight;

    public VisionSubsystem() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    // ==================== BASIC TARGET DATA ====================

    /** Returns true if a valid target is detected */
    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    /** Horizontal angle to target in degrees (-27 to 27) */
    public double getTargetX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    /** Vertical angle to target in degrees (-20.5 to 20.5) */
    public double getTargetY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    /** Target area (0% to 100% of image) */
    public double getTargetArea() {
        return limelight.getEntry("ta").getDouble(0);
    }

    /** AprilTag ID (-1 if none) */
    public int getAprilTagID() {
        return (int) limelight.getEntry("tid").getDouble(-1);
    }

    // ==================== AI OBJECT DETECTION (HAILO-8) ====================

    /** Returns the class name of detected object (e.g., "coral", "algae") */
    public String getDetectedClass() {
        return limelight.getEntry("tclass").getString("none");
    }

    /** Returns true if a specific game piece is detected */
    public boolean isGamePieceDetected(String className) {
        return hasTarget() && getDetectedClass().equals(className);
    }

    // ==================== ROBOT LOCALIZATION ====================

    /** Returns robot pose from AprilTag detection */
    public Pose2d getRobotPose() {
        double[] botpose = limelight.getEntry("botpose").getDoubleArray(new double[6]);
        return new Pose2d(
            botpose[0],  // x (meters)
            botpose[1],  // y (meters)
            Rotation2d.fromDegrees(botpose[5])  // yaw (degrees)
        );
    }

    // ==================== PIPELINE CONTROL ====================

    /** Switch between vision pipelines (0-9) */
    public void setPipeline(int pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline);
    }

    /** Set LED mode: 0=pipeline, 1=off, 2=blink, 3=on */
    public void setLEDMode(int mode) {
        limelight.getEntry("ledMode").setNumber(mode);
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
        SmartDashboard.putNumber("Vision/TargetX", getTargetX());
        SmartDashboard.putNumber("Vision/TargetY", getTargetY());
        SmartDashboard.putNumber("Vision/AprilTagID", getAprilTagID());
        SmartDashboard.putString("Vision/DetectedClass", getDetectedClass());
    }
}