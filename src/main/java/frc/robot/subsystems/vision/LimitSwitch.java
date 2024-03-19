package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimitSwitch extends GenericDigitalPinObject
{
    private double channel;

    public LimitSwitch(int channel){
       super(channel);
       this.channel = channel;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Limit Switch " + channel, this.getStatus());
    }

}