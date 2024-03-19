package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

public class elevatorPosition extends Command{

    Timer timer = new Timer();
    private Elevator m_elevator;
    private ElevatorPositions targetPosition;
    private boolean isFinished;


    public elevatorPosition(Elevator elevator, ElevatorPositions position){
        m_elevator = elevator;
        this.targetPosition = position;
        // addRequirements(m_elevator);
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        System.out.println(targetPosition);
        m_elevator.setElevatorPosition(targetPosition);
        m_elevator.setPivotPosition(targetPosition);
        
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
       return (Math.abs(m_elevator.getElevatorEncoderPosition() - m_elevator.getTargetElevatorPositionEncoderValue(targetPosition)) < 1) || timer.get()>2;
    }
}