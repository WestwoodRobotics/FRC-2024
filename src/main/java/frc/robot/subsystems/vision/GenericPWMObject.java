package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GenericPWMObject extends SubsystemBase{
    private DigitalInput input;

    public GenericPWMObject(int channel){
       input = new DigitalInput(channel);
    }

    public boolean getStatus(){
        return (input.get());
    }

    @Override
    public void periodic(){
    }
}