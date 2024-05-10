package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakeShooter;

/**
 * Command to shoot for a specified duration.
 * This command activates the intake shooter subsystem to shoot for a specified ramp-up duration and power level.
 */
public class ShootForTimeCommand extends Command{
    Timer timer = new Timer();
    private IntakeShooter intakeShooterSubsystem; // Renamed for clarity
    private double rampUpDuration; // Renamed for clarity
    private double shootingPower; // Renamed for clarity

    private boolean isFinished;

    /**
     * Constructs a ShootForTimeCommand.
     * @param intakeShooterSubsystem The intake shooter subsystem to be controlled
     * @param rampUpDuration The duration to ramp up the shooting mechanism
     * @param shootingPower The power level for shooting
     */
    public ShootForTimeCommand(IntakeShooter intakeShooterSubsystem, double rampUpDuration, double shootingPower){
        this.intakeShooterSubsystem = intakeShooterSubsystem;
        this.rampUpDuration = rampUpDuration;
        this.shootingPower = shootingPower;
    }

    /**
     * Initializes the command with a timer reset.
     */
    @Override
    public void initialize(){
        timer.reset(); 
        timer.start();
    }

    /**
     * Executes the command, activating the intake shooter for the specified duration and power.
     */
    @Override
    public void execute(){
        intakeShooterSubsystem.setRollerPower(-shootingPower);
        Timer.delay(rampUpDuration);
        intakeShooterSubsystem.setStowPower(shootingPower);
        Timer.delay(2);
        isFinished = true;
    }

    /**
     * Determines if the command has finished.
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
        return isFinished;
    }

    /**
     * Ends the command, stopping the intake shooter.
     * @param interrupted Whether the command was interrupted or not
     */
    @Override
    public void end(boolean interrupted){
        intakeShooterSubsystem.setRollerPower(0);
        intakeShooterSubsystem.setStowPower(0);
    }
}
