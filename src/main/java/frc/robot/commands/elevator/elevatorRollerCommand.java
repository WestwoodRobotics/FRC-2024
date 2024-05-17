package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.BeamBreak;

public class elevatorRollerCommand extends Command{
    Timer t = new Timer();
    private Elevator elevator;
    private double rollerPower;
    private BeamBreak elevatorBeamBreak;
    private Boolean alreadyBeamBreak;
    private double timerStart;


    private boolean isFinished;

    public elevatorRollerCommand(Elevator elevator, double rollerPower, BeamBreak elevatorBeamBreak){
        this.elevator = elevator;
        this.rollerPower = rollerPower;
        this.elevatorBeamBreak = elevatorBeamBreak;
        //addRequirements(intakeShooter);
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){
        t.reset(); 
        t.start();
        isFinished = false;
        alreadyBeamBreak = elevatorBeamBreak.getStatus();
        // if(alreadyBeamBreak){
        //     timerStart = t.get();
        //     System.out.println("broken");
        // }
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Checks if the motors are at their target RPM and either finishes the command or resets the timer.
     */
    @Override
    public void execute(){
        if(alreadyBeamBreak){
            timerStart = t.get();
        }
        if((t.get() - timerStart) >= 0.1){
            elevator.setRollerPower(0);
            isFinished = true;
        }
        else{
            elevator.setRollerPower(rollerPower);
        }

        if(elevatorBeamBreak.getStatus() && !alreadyBeamBreak){
            alreadyBeamBreak = true;
        }
        else{
            alreadyBeamBreak = false;
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted){
        elevator.setRollerPower(0);
    }



    
}