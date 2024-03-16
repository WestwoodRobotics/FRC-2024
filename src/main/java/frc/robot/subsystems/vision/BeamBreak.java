package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreak extends GenericDigitalPinObject{
    private DigitalInput input;

    public BeamBreak(int channel){
       super(channel);
    }
}