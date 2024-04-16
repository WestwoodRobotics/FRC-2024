package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase
{   
    private final NetworkTable networkTable;
    private NetworkTableEntry detected;
    private NetworkTableEntry targetHorizontalDiffAngle;
    private NetworkTableEntry targetVerticalDiffAngle;
    private NetworkTableEntry targetArea;
    private NetworkTableEntry hardwareMetrics;

    public Vision()
    {
        // Add tid (april tag id) field
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        detected = networkTable.getEntry("tv");
        targetHorizontalDiffAngle = networkTable.getEntry("tx");
        targetVerticalDiffAngle = networkTable.getEntry("ty");
        targetArea = networkTable.getEntry("ta");
        hardwareMetrics = networkTable.getEntry("hw");
    }

    /*
     * Returns true if the limelight has detected a target
     * 
     */
    public boolean found(){
        return(detected.getDouble(0.0) == 1);
    }
    
    /*
     * Returns the horizontal difference angle between 
     * The target April Tag and the crosshair
     */
    public double getHorizontalDiff()
    {
        return targetHorizontalDiffAngle.getDouble(-1.0);
    }

    public double getVerticalDiff() {
        return targetVerticalDiffAngle.getDouble(-1.0);
    }

    public double getTargetArea() {
        if (this.found()) {
            return targetArea.getDouble(0.0);
        }
        else {
            return -1;
        }
    }

    public double[] getHardwareMetrics() {
        return hardwareMetrics.getDoubleArray(new double[0]);
    }
    
    public double getFPS(){
        double[] array = this.getHardwareMetrics();
        return array[0];
    }

    public void setPipeline(int pipeline) {
        networkTable.getEntry("pipeline").setNumber(pipeline);
    }


    


}