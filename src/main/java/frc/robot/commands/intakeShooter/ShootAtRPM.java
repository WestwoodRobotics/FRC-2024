package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakeShooter;


public class ShootAtRPM extends Command{
    private Timer timer;
    private IntakeShooter m_intakeShooter;
    private double targetUpperRPM;
    private double targetLowerRPM;
    private boolean isFinished;




    public ShootAtRPM(IntakeShooter intakeShooter, double targetUpperRPM, double targetLowerRPM){
        timer = new Timer();
        timer.start();
        m_intakeShooter = intakeShooter;
        this.targetUpperRPM = targetUpperRPM;
        this.targetLowerRPM = targetLowerRPM;
        m_intakeShooter.setRollerRPM(targetUpperRPM, targetLowerRPM);
    }

    @Override
    public void execute(){
        // Check if the motors are at their target RPM
        if (Math.abs(m_intakeShooter.getUpperRPM() - targetUpperRPM) < 100.0 && Math.abs(m_intakeShooter.getLowerRPM() - targetLowerRPM) < 100.0) {
            // If they are at their target RPM for more than 1 second
            if (timer.get() > 1.0) {
                // Turn on the stow motor for 1 second
                m_intakeShooter.setStowPower(1.0);
                Timer.delay(1.0); // Wait for 1 second
                // Then set the power of all motors to zero
                m_intakeShooter.setRollerPower(0.0);
                m_intakeShooter.setPivotPower(0.0);
                isFinished = true;
            }
        } else {
            // Reset the timer if the motors are not at their target RPM
            timer.reset();
            timer.start();
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    
}
