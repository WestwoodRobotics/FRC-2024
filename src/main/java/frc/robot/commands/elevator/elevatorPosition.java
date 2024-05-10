package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;

/**
 * Command to set the elevator to a specific position.
 * This command sets the elevator subsystem to a desired position based on the ElevatorPositions enum.
 */
public class elevatorPosition extends Command{

    Timer timer = new Timer();
    private Elevator elevatorSubsystem; // Renamed for clarity
    private ElevatorPositions desiredElevatorPosition; // Renamed for clarity
    private boolean hasCommandFinished; // Renamed for clarity

    /**
     * Constructor for elevatorPosition command.
     * @param elevatorSubsystem The elevator subsystem to be controlled
     * @param desiredElevatorPosition The target position for the elevator
     */
    public elevatorPosition(Elevator elevatorSubsystem, ElevatorPositions desiredElevatorPosition){
        this.elevatorSubsystem = elevatorSubsystem;
        this.desiredElevatorPosition = desiredElevatorPosition;
        // addRequirements(elevatorSubsystem);
    }

    /**
     * Initializes the command with a timer reset and starts the elevator movement.
     */
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        System.out.println(desiredElevatorPosition);
        elevatorSubsystem.setElevatorPosition(desiredElevatorPosition);
        elevatorSubsystem.setPivotPosition(desiredElevatorPosition);
        
    }

    /**
     * Executes the command, primarily used for monitoring or additional logic.
     */
    @Override
    public void execute(){
        // Additional execution logic can be added here if needed
    }

    /**
     * Determines if the command has finished executing.
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       return timer.get()>2; // Uses the timer to determine if the command has finished
    }
}
