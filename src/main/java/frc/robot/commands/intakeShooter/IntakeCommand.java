package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

public class IntakeCommand extends Command{
    Timer t = new Timer();
    private IntakeShooter m_intakeShooter;
    private double rollerPower;
    private double stowPower;


    private boolean isFinished;

    public IntakeCommand(IntakeShooter intakeShooter, double rollerPower, double stowPower){
        m_intakeShooter = intakeShooter;
        this.rollerPower = rollerPower;
        this.stowPower = stowPower;
        addRequirements(intakeShooter);
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){
        t.reset(); 
        t.start();
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Checks if the motors are at their target RPM and either finishes the command or resets the timer.
     */
    @Override
    public void execute(){
        m_intakeShooter.setRollerPower(rollerPower);
        m_intakeShooter.setStowPower(stowPower);

        if(t.get() > 1){
            m_intakeShooter.setToPosition(IntakeShooterPositions.INTAKE);
            isFinished = true;
        }

    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }



    
}