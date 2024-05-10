package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

/**
 * Command to shoot balls at a specified RPM using the intake shooter subsystem.
 * This command will attempt to reach and maintain the desired RPM for both upper and lower rollers of the intake shooter.
 */
public class ShootAtRPM extends Command{
    private Timer timer;
    private double rpmToleranceDuration; // Renamed for clarity
    private IntakeShooter intakeShooterSubsystem; // Renamed for clarity
    private double desiredUpperRPM; // Renamed for clarity
    private double desiredLowerRPM; // Renamed for clarity
    private boolean isFinished;

    /**
     * Constructor with default timer tolerance of 1 second.
     * @param intakeShooterSubsystem The IntakeShooter subsystem this command will run on.
     * @param desiredUpperRPM The target RPM for the upper motor.
     * @param desiredLowerRPM The target RPM for the lower motor.
     */
    public ShootAtRPM(IntakeShooter intakeShooterSubsystem, double desiredUpperRPM, double desiredLowerRPM){
        timer = new Timer();
        timer.start();
        this.intakeShooterSubsystem = intakeShooterSubsystem;
        this.desiredUpperRPM = desiredUpperRPM;
        this.desiredLowerRPM = desiredLowerRPM;
        this.rpmToleranceDuration = 1; //Default timer tolerance of 1 second.
    }

    /**
     * Constructor with custom timer tolerance.
     * @param intakeShooterSubsystem The IntakeShooter subsystem this command will run on.
     * @param desiredUpperRPM The target RPM for the upper motor.
     * @param desiredLowerRPM The target RPM for the lower motor.
     * @param rpmToleranceDuration The time in seconds the motors need to be at their target RPM before the command finishes.
     */
    public ShootAtRPM(IntakeShooter intakeShooterSubsystem, double desiredUpperRPM, double desiredLowerRPM, double rpmToleranceDuration){
        timer = new Timer();
        timer.start();
        this.intakeShooterSubsystem = intakeShooterSubsystem;
        this.desiredUpperRPM =  desiredUpperRPM;
        this.desiredLowerRPM = desiredLowerRPM;
        this.rpmToleranceDuration = rpmToleranceDuration;
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){
        intakeShooterSubsystem.setRollerRPM(desiredUpperRPM, desiredLowerRPM);
        timer.reset();
        timer.start();
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Checks if the motors are at their target RPM and either finishes the command or resets the timer.
     */
    @Override
    public void execute(){
        if (((Math.abs(intakeShooterSubsystem.getUpperRPM()) - Math.abs(desiredUpperRPM) < 100.0) 
        && (Math.abs(intakeShooterSubsystem.getLowerRPM()) - Math.abs(desiredLowerRPM) < 100.0 )) 
        || (timer.get() > rpmToleranceDuration)) {
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
        return isFinished;
    }

    public void end (boolean interrupted){

    }
}
