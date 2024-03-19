package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.BeamBreak;
import frc.robot.subsystems.vision.LimitSwitch;

public class IntakeBeamBreakCommand extends Command{
    Timer t = new Timer();
    private IntakeShooter m_intakeShooter;
    // private double rollerPower;
    // private double stowPower;
    // private boolean manualOverride;
    private BeamBreak l;
    private IntakeShooterPositions i;



    private boolean isFinished;

    public IntakeBeamBreakCommand(IntakeShooter intakeShooter, BeamBreak l){
        m_intakeShooter = intakeShooter;
        this.l = l;
        this.i = i;
        // this.rollerPower = rollerPower;
        // this.stowPower = stowPower;
        // this.manualOverride = manualOverride;
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
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Checks if the motors are at their target RPM and either finishes the command or resets the timer.
     */
    @Override
    public void execute(){
        m_intakeShooter.setRollerPower(1);
        m_intakeShooter.setStowPower(-1);
        isFinished = false;
    }

    @Override
    public boolean isFinished(){
        return !l.getStatus();
    }

    @Override
    public void end(boolean interrupted){
        m_intakeShooter.setRollerPower(0);
        m_intakeShooter.setStowPower(0);
    }



    
}