package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

public class ShootAtRPM extends Command{
    private Timer timer;
    private double timerTolerance;
    private IntakeShooter m_intakeShooter;
    private double targetUpperRPM;
    private double targetLowerRPM;
    private boolean isFinished;

    /**
     * Constructor with default timer tolerance of 1 second.
     * @param intakeShooter The IntakeShooter subsystem this command will run on.
     * @param targetUpperRPM The target RPM for the upper motor.
     * @param targetLowerRPM The target RPM for the lower motor.
     */
    public ShootAtRPM(IntakeShooter intakeShooter, double targetUpperRPM, double targetLowerRPM){
        timer = new Timer();
        timer.start();
        m_intakeShooter = intakeShooter;
        this.targetUpperRPM = targetUpperRPM;
        this.targetLowerRPM = targetLowerRPM;
        this.timerTolerance = 1; //Default timer tolerance of 1 second.
    }

    /**
     * Constructor with custom timer tolerance.
     * @param intakeShooter The IntakeShooter subsystem this command will run on.
     * @param targetUpperRPM The target RPM for the upper motor.
     * @param targetLowerRPM The target RPM for the lower motor.
     * @param timerTolerance The time in seconds the motors need to be at their target RPM before the command finishes.
     */
    public ShootAtRPM(IntakeShooter intakeShooter, double targetUpperRPM, double targetLowerRPM, double timerTolerance){
        timer = new Timer();
        timer.start();
        m_intakeShooter = intakeShooter;
        this.targetUpperRPM =  targetUpperRPM;
        this.targetLowerRPM = targetLowerRPM;
        this.timerTolerance = timerTolerance;
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){
        // if (m_intakeShooter.getState() == IntakeShooterPositions.SHOOT_FAR_SPEAKER || m_intakeShooter.getState() == IntakeShooterPositions.SHOOT_NEAR_SPEAKER){
        //     m_intakeShooter.setRollerRPM(targetUpperRPM, targetLowerRPM);
        // }
        // else{
        //     System.out.println("Intake Shooter State is not SHOOT_FAR_SPEAKER or SHOOT_NEAR_SPEAKER (Current State: " + m_intakeShooter.getState().toString() + ")");
        //     isFinished = true;
        // }
        m_intakeShooter.setRollerRPM(targetUpperRPM, targetLowerRPM);
        timer.reset();
        timer.start();

    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Checks if the motors are at their target RPM and either finishes the command or resets the timer.
     */
    @Override
    public void execute(){
        if (((Math.abs(m_intakeShooter.getUpperRPM()) - Math.abs(targetUpperRPM) < 100.0) 
        && (Math.abs(m_intakeShooter.getLowerRPM()) - Math.abs(targetLowerRPM) < 100.0 )) 
        || (timer.get() > timerTolerance)) {
            {
                isFinished = true;
            }
        } else {
            timer.reset();
            timer.start();
        }
    }

    /**
     * Returns whether the command has finished.
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
        return true;
    }

    public void end (boolean interrupted){

    }
}
