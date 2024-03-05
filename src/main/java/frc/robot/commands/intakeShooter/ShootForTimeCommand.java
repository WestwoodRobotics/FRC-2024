package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

public class ShootForTimeCommand extends Command{
    Timer t = new Timer();
    private IntakeShooter m_intakeShooter;
    private double rampUpTime;


    private boolean isFinished;

    public ShootForTimeCommand(IntakeShooter intakeShooter, double rampUpTime){
        m_intakeShooter = intakeShooter;
        this.rampUpTime = rampUpTime;
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
        m_intakeShooter.setToPosition(IntakeShooterPositions.SHOOT_NEAR_SPEAKER);
        Timer.delay(0.7);
        m_intakeShooter.setRollerPower(1);
        Timer.delay(rampUpTime);
        m_intakeShooter.setStowPower(1);
        Timer.delay(1);
        isFinished = true;
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted){
        m_intakeShooter.setRollerPower(0);
        m_intakeShooter.setStowPower(0);
    }



    
}