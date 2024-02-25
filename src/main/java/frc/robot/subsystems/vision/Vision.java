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

    public Vision()
    {
        // Add tid (april tag id) field
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        detected = networkTable.getEntry("tv");
        targetHorizontalDiffAngle = networkTable.getEntry("tx");
        targetVerticalDiffAngle = networkTable.getEntry("ty");
        targetArea = networkTable.getEntry("ta");
    }

    public boolean found(){
        return(detected.getDouble(0.0) == 1);
    }
    
    public double getHorizontalDiff()
    {
        return targetHorizontalDiffAngle.getDouble(0.0);
    }

    public double getVerticalDiff() {
        return targetVerticalDiffAngle.getDouble(0.0);
    }

    public double getTargetArea() {
        if (this.found()) {
            return targetArea.getDouble(0.0);
        }
        else {
            return -1;
        }
    }
}