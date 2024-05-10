package frc.robot.commands.utils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeRollers;

public class StopAllRollersCommand extends Command {
     private IntakeRollers m_intakeShooterRollers;
     private Elevator m_elevator;

    public StopAllRollersCommand(IntakeRollers intakeShooter, Elevator elevator){
        m_intakeShooterRollers = intakeShooter;
        m_elevator = elevator;
        addRequirements(intakeShooter, elevator);
    }
    
    @Override
    public void execute(){
        m_intakeShooterRollers.setRollerPower(0);
        m_intakeShooterRollers.setStowPower(0);
        m_elevator.setRollerPower(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override 
    public void end(boolean interrupted){
        m_intakeShooterRollers.setRollerPower(0);
        m_intakeShooterRollers.setStowPower(0);
        m_elevator.setRollerPower(0);
    }
}