package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.BeamBreak;

/**
 * Command to control the elevator roller based on beam break sensor input.
 * This command controls the roller of the elevator subsystem, activating it based on the status of a beam break sensor.
 */
public class elevatorRollerCommand extends Command{
    Timer t = new Timer();
    private Elevator elevatorSubsystem; // Renamed for clarity
    private double desiredRollerPower; // Renamed for clarity
    private BeamBreak beamBreakSensor; // Renamed for clarity
    private Boolean beamBreakTriggered; // Renamed for clarity
    private double timerStart;

    private boolean isFinished;

    /**
     * Constructs an elevatorRollerCommand.
     * @param elevatorSubsystem The elevator subsystem to be controlled
     * @param desiredRollerPower The power to set for the roller
     * @param beamBreakSensor The beam break sensor to monitor
     */
    public elevatorRollerCommand(Elevator elevatorSubsystem, double desiredRollerPower, BeamBreak beamBreakSensor){
        this.elevatorSubsystem = elevatorSubsystem;
        this.desiredRollerPower = desiredRollerPower;
        this.beamBreakSensor = beamBreakSensor;
    }

    /**
     * Initializes the command with a timer reset.
     */
    @Override
    public void initialize(){
        t.reset(); 
        t.start();
        isFinished = false;
        beamBreakTriggered = beamBreakSensor.getStatus();
    }

    /**
     * Executes the command, controlling the roller based on the beam break sensor.
     */
    @Override
    public void execute(){
        if(beamBreakTriggered){
            timerStart = t.get();
        }
        if((t.get() - timerStart) >= 0.1){
            elevatorSubsystem.setRollerPower(0);
            isFinished = true;
        }
        else{
            elevatorSubsystem.setRollerPower(desiredRollerPower);
        }

        if(beamBreakSensor.getStatus() && !beamBreakTriggered){
            beamBreakTriggered = true;
        }
        else{
            beamBreakTriggered = false;
        }
    }

    /**
     * Determines if the command has finished.
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
        return isFinished;
    }

    /**
     * Ends the command, stopping the roller.
     * @param interrupted Whether the command was interrupted or not
     */
    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.setRollerPower(0);
    }
}
