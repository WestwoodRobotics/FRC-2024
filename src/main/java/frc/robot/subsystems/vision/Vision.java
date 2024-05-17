package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Vision subsystem interfaces with the Limelight camera to detect targets and provide target information.
 * It provides methods to check if a target is detected, get target position and area, and set the camera pipeline.
 */
public class Vision extends SubsystemBase
{   
    private final NetworkTable networkTable;
    private NetworkTableEntry targetDetected;
    private NetworkTableEntry targetHorizontalDiffAngle;
    private NetworkTableEntry targetVerticalDiffAngle;
    private NetworkTableEntry targetArea;
    private NetworkTableEntry hardwareMetrics;
    private NetworkTableEntry aprilTagTargetTransformArray;

    /**
     * Constructs the Vision subsystem and initializes network table entries for target detection and metrics.
     */
    public Vision()
    {
        // Initialize the network table for Limelight and retrieve entries for target detection and metrics
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        targetDetected = networkTable.getEntry("tv");
        targetHorizontalDiffAngle = networkTable.getEntry("tx");
        targetVerticalDiffAngle = networkTable.getEntry("ty");
        targetArea = networkTable.getEntry("ta");
        hardwareMetrics = networkTable.getEntry("hw");
        aprilTagTargetTransformArray = networkTable.getEntry("botpose_targetspace");
    }

    /**
     * Checks if the Limelight has detected a target.
     * 
     * @return true if a target is detected, false otherwise.
     */
    public boolean found(){
        return(targetDetected.getDouble(0.0) == 1);
    }
    
    /**
     * Gets the horizontal difference angle between the target and the crosshair.
     * 
     * @return The horizontal difference angle in degrees.
     */
    public double getHorizontalDiff()
    {
        return targetHorizontalDiffAngle.getDouble(-1.0);
    }

    /**
     * Gets the vertical difference angle between the target and the crosshair.
     * 
     * @return The vertical difference angle in degrees.
     */
    public double getVerticalDiff() {
        return targetVerticalDiffAngle.getDouble(-1.0);
    }

    /**
     * Gets the area of the detected target as a percentage of the total image area.
     * 
     * @return The target area percentage, or -1 if no target is found.
     */
    public double getTargetArea() {
        if (this.found()) {
            return targetArea.getDouble(0.0);
        }
        else {
            return -1;
        }
    }

    /**
     * Retrieves hardware metrics from the Limelight.
     * 
     * @return An array of hardware metrics, including FPS.
     */
    public double[] getHardwareMetrics() {
        return hardwareMetrics.getDoubleArray(new double[0]);
    }
    
    /**
     * Gets the frames per second (FPS) the Limelight camera is operating at.
     * 
     * @return The current FPS.
     */
    public double getFPS(){
        double[] array = this.getHardwareMetrics();
        return array[0];
    }

    /**
     * Returns the transform of the target April Tag.
     * The transform includes horizontal and vertical difference, distance, pitch, yaw, and roll.
     * 
     * @return An array containing the transform [tx, ty, tz, pitch, yaw, roll].
     */
    public double[] getAprilTagTransform(){
        return aprilTagTargetTransformArray.getDoubleArray(new double[0]);
    }

    /**
     * Sets the pipeline of the Limelight camera.
     * 
     * @param pipeline The pipeline number to set.
     */
    public void setPipeline(int pipeline) {
        networkTable.getEntry("pipeline").setNumber(pipeline);
    }
}
