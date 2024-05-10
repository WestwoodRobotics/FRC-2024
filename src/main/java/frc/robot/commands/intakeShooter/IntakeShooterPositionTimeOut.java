package frc.robot.commands.intakeShooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakePivot;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LimitSwitch;

public class IntakeShooterPositionTimeOut extends Command{
    Timer t = new Timer();
    private IntakePivot m_intakePivot;
    private IntakeShooterPositions targetPosition;
    private double timeOutPeriod;
    private LimitSwitch l;


    /*
     * Constructor for the IntakeShooterPosition command.
     * @param intakePivot The subsystem used by this command. (IntakePivot)
     * @param position The target position for the IntakeShooter. (IntakeShooterPositions)
     * @param limitSwitch The limit switch used to determine if the pivot is at the home position. (LimitSwitch)
     * @param timeOutPeriod The time out period for the command. (double)
     */
    public IntakeShooterPositionTimeOut(IntakePivot intakePivot, IntakeShooterPositions position, LimitSwitch limitSwitch, double timeOutPeriod
    ){
        this.m_intakePivot = intakePivot;
        this.targetPosition = position;
        this.timeOutPeriod = timeOutPeriod;
        addRequirements(m_intakePivot);
        this.l = limitSwitch;
    }


    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){
        // m_intakePivot.setRollerPower(0);
        t.reset();
        t.start();
        
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Checks if the motors are at their target RPM and either finishes the command or resets the timer.
     */
    @Override
    public void execute(){
       //m_elevator.setElevatorPositionNOPID(targetPosition);
        //    m_elevator.setPivotPosition(targetPosition);

        if(targetPosition == IntakeShooterPositions.HOME){
            if(!m_intakePivot.getPivotLimitReached()){
                m_intakePivot.setPivotPower(-0.75);
            }
            else{
                m_intakePivot.setPivotPower(0);
                m_intakePivot.resetEncoder();
            }
        }
        // else if(targetPosition == IntakeShooterPositions.AUTON_SHOOT){
        //     if(m_intakeShooter.getPivotLimitReached() && !isAlreadyPressed){
        //         m_intakeShooter.setPivotPower(0);
        //         m_intakeShooter.resetEncoder();
        //         isFinished = true;
        //     }
        //     else{
        //         isFinished = m_intakeShooter.setToPosition(targetPosition);
        //     }
        // }
        else{    
            m_intakePivot.setToPosition(targetPosition);

            // if(changePower){
            //     if (targetPosition == IntakeShooterPositions.SHOOT_FAR_SPEAKER || targetPosition == IntakeShooterPositions.SHOOT_NEAR_SPEAKER || targetPosition == IntakeShooterPositions.AUTON_SHOOT || targetPosition == IntakeShooterPositions.SHOOT_NEAR_SPEAKER_FACING_FORWARDS){
            //         m_intakePivot.setRollerPower(-1);
            //         m_intakePivot.setStowPower(0);
                    
            //     }  
            //     else{
            //         m_intakePivot.setRollerPower(0);
            //         m_intakePivot.setStowPower(0);
            //     }
            // }   
        }
    }

    /**
     * Returns whether the command has finished.
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       return (t.get() > timeOutPeriod) || (l.getStatus() && (targetPosition == (IntakeShooterPositions.HOME)));
    }

    @Override
    public void end(boolean interrupted){
        //m_intakeShooter.setRollerPower(0);
        //m_intakeShooter.setStowPower(0);
        if(!interrupted){
            m_intakePivot.setPivotPower(0);
        }

        m_intakePivot.setPositionState( interrupted ? IntakeShooterPositions.MANUAL : targetPosition);
        System.out.println("finished!!");
    }
    
}