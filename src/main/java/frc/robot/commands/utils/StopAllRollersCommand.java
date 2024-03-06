package frc.robot.commands.utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;

public class StopAllRollersCommand extends Command {
     private IntakeShooter m_intakeShooter;
     private Elevator m_elevator;

    public StopAllRollersCommand(IntakeShooter intakeShooter, Elevator elevator){
        m_intakeShooter = intakeShooter;
        m_elevator = elevator;

        addRequirements(intakeShooter, elevator);
    }
    
    @Override
    public void execute(){
        m_intakeShooter.setRollerPower(0);
        m_intakeShooter.setStowPower(0);
        m_elevator.setRollerPower(0);
    }
}