package frc.robot.commands.vision;

import frc.robot.subsystems.vision.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDCommand extends Command {

    private LED led;
    private BeamBreak beamBreakIntakePivot, beamBreakElevatorPivot;


    public LEDCommand(LED i, BeamBreak IntakeShooterBeamBreak, BeamBreak ElevatorBeamBreak){
        led = i;
        addRequirements(i);
        beamBreakIntakePivot = IntakeShooterBeamBreak;
        beamBreakElevatorPivot = ElevatorBeamBreak;
    }


    @Override
    public void execute(){        
        // if(!b.getStatus()) {
        //     led.setLights(0.77);
        // } else {
        //     led.setLights(0.61);
        // }
        // SmartDashboard.putBoolean("b", b.getStatus());
        SmartDashboard.putBoolean("elevator beam break", beamBreakElevatorPivot.getStatus());
        SmartDashboard.putBoolean("intake shooter beam break", beamBreakIntakePivot.getStatus());
        if(!beamBreakIntakePivot.getStatus() || !beamBreakElevatorPivot.getStatus()){
            //if(!beamBreakIntakePivot.getStatus()){
            led.setLights(0.77);
            } else {
            led.setLights(0.61);
            }

    }
    
    
}