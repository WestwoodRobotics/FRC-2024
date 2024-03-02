package frc.robot.commands.vision;

import frc.robot.subsystems.vision.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDCommand extends Command {

    private LED led;
    private BeamBreak b;

    public LEDCommand(LED i, BeamBreak beam){
        led = i;
        addRequirements(i);
        b = beam;
    }

    @Override
    public void execute(){        
        if(!b.getStatus()) {
            led.setLights(0.77);
        } else {
            led.setLights(0.61);
        }
        SmartDashboard.putBoolean("b", b.getStatus());
    }
    
    
}