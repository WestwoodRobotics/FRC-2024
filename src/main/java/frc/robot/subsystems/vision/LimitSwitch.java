package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimitSwitch extends GenericDigitalPinObject
{
    private DigitalInput input;

    public LimitSwitch(int channel){
       super(channel);
    }

}
