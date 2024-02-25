package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakeShooter;


public class intakeShooterPID extends Command{
    private Timer timer;
    private IntakeShooter m_intakeShooter;
    private double targetUpperRPM;
    private double targetLowerRPM;




    public intakeShooterPID(IntakeShooter intakeShooter, double targetUpperRPM, double targetLowerRPM){
        timer = new Timer();
        timer.start();
        m_intakeShooter = intakeShooter;
        this.targetUpperRPM = targetUpperRPM;
        this.targetLowerRPM = targetLowerRPM;
        m_intakeShooter.setRollerRPM(targetUpperRPM, targetLowerRPM);

    }

    @Override
    public void execute(){
        // Run the Shooter motors to the target RPM
        m_intakeShooter.setRollerRPM(targetUpperRPM, targetLowerRPM);
    
        // Check if the motors are at their target RPM
        if (Math.abs(m_intakeShooter.getUpperRPM() - targetUpperRPM) < 1.0 && Math.abs(m_intakeShooter.getLowerRPM() - targetLowerRPM) < 1.0) {
            // If they are at their target RPM for more than 1 second
            if (timer.get() > 1.0) {
                // Turn on the pivot motor for 1 second
                m_intakeShooter.setPivotPower(1.0);
                Timer.delay(1.0); // Wait for 1 second
                // Then set the power of all motors to zero
                m_intakeShooter.setRollerPower(0.0);
                m_intakeShooter.setPivotPower(0.0);
            }
        } else {
            // Reset the timer if the motors are not at their target RPM
            timer.reset();
            timer.start();
        }
    }

    @Override
    public boolean isFinished(){
        // The command is finished if the motors have reached their target RPM and the pivot motor has been turned on for 1 second
        return Math.abs(m_intakeShooter.getUpperRPM() - targetUpperRPM) < 1.0 
            && Math.abs(m_intakeShooter.getLowerRPM() - targetLowerRPM) < 1.0 
            && timer.get() > 1.0;
    }

    
}
