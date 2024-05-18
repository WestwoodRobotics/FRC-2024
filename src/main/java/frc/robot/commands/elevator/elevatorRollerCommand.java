package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.BeamBreak;

public class elevatorRollerCommand extends Command{
    Timer timer = new Timer();
    private Elevator elevatorSubsystem;
    private double elevatorRollerPower;
    private BeamBreak beamBreakSensor;
    private Boolean isBeamBrokenPreviously;
    private double beamBreakTimerStart;


    private boolean isCommandFinished;

    public elevatorRollerCommand(Elevator elevator, double rollerPower, BeamBreak elevatorBeamBreak){
        this.elevatorSubsystem = elevator;
        this.elevatorRollerPower = rollerPower;
        this.beamBreakSensor = elevatorBeamBreak;
        //addRequirements(intakeShooter);
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){
        timer.reset(); 
        timer.start();
        isCommandFinished = false;
        isBeamBrokenPreviously = beamBreakSensor.getStatus();
        // if(isBeamBrokenPreviously){
        //     beamBreakTimerStart = timer.get();
        //     System.out.println("broken");
        // }
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Checks if the motors are at their target RPM and either finishes the command or resets the timer.
     */
    @Override
    public void execute(){
        if(isBeamBrokenPreviously){
            beamBreakTimerStart = timer.get();
        }
        if((timer.get() - beamBreakTimerStart) >= 0.1){
            elevatorSubsystem.setRollerPower(0);
            isCommandFinished = true;
        }
        else{
            elevatorSubsystem.setRollerPower(elevatorRollerPower);
        }

        if(beamBreakSensor.getStatus() && !isBeamBrokenPreviously){
            isBeamBrokenPreviously = true;
        }
        else{
            isBeamBrokenPreviously = false;
        }
    }

    @Override
    public boolean isFinished(){
        return isCommandFinished;
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.setRollerPower(0);
    }



    
}
