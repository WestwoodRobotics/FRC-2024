package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;


public class elevatorPosition extends Command{

    Timer elevatorTimer = new Timer();
    private Elevator elevatorSubsystem;
    private ElevatorPositions elevatorTargetPosition;


    public elevatorPosition(Elevator elevator, ElevatorPositions position){
        elevatorSubsystem = elevator;
        this.elevatorTargetPosition = position;
        // addRequirements(m_elevator);
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){
        elevatorTimer.reset();
        elevatorTimer.start();
        System.out.println(elevatorTargetPosition);
        elevatorSubsystem.setElevatorPosition(elevatorTargetPosition);
        elevatorSubsystem.setPivotPosition(elevatorTargetPosition);
        
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Checks if the motors are at their target RPM and either finishes the command or resets the timer.
     */
    @Override
    public void execute(){
        
        //m_elevator.setPivotPosition(targetPosition);
    }

    /**
     * Returns whether the command has finished.
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       return elevatorTimer.get()>2;
    }
}
