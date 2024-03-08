package frc.robot.commands.intakeShooter;

import java.util.concurrent.SynchronousQueue;

import com.revrobotics.CANSparkBase.IdleMode;

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
    private double power;


    private boolean isFinished;

    public ShootForTimeCommand(IntakeShooter intakeShooter, double rampUpTime, double power){
        m_intakeShooter = intakeShooter;
        m_intakeShooter.pivotMotor.setIdleMode(IdleMode.kBrake);
        this.rampUpTime = rampUpTime;
        this.power = power;

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
        // m_intakeShooter.setToPosition(IntakeShooterPositions.SHOOT_NEAR_SPEAKER);
        // Timer.delay(0.7);
        System.out.println("geot here");
        m_intakeShooter.setRollerPower(-power);
        Timer.delay(rampUpTime);
        m_intakeShooter.setStowPower(power);
        Timer.delay(2);
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