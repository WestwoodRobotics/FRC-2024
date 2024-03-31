package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeRollers;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

public class IntakeRollersCommand extends Command{
    Timer t = new Timer();
    private IntakeRollers m_intakeRollers;
    private double rollerPower;
    private double stowPower;
    

    public IntakeRollersCommand(IntakeRollers intakeRollers, double rollerPower, double stowPower){
        m_intakeRollers = intakeRollers;
        this.rollerPower = rollerPower;
        this.stowPower = stowPower;
        addRequirements(intakeRollers);

        //addRequirements(intakeShooter);
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){
        m_intakeRollers.setRollerPower(rollerPower);
        m_intakeRollers.setStowPower(stowPower);
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Checks if the motors are at their target RPM and either finishes the command or resets the timer.
     */
    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }



    
}