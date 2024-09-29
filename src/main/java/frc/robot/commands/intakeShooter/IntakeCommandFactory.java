package frc.robot.commands.intakeShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intakeShooter.IntakePivot;
import frc.robot.subsystems.intakeShooter.IntakeRollers;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

/**
 * Factory class for creating intake and shooter commands.
 * This class provides methods to create commands that combine actions on the intake pivot and rollers.
 */
public class IntakeCommandFactory {
    
    /**
     * Creates a command to move the intake pivot to the shooting position and activate the rollers to shoot.
     * @param intakePivot The intake pivot subsystem.
     * @param intakeRollers The intake rollers subsystem.
     * @return A command that moves the intake pivot and activates the rollers for shooting.
     */
    public Command goToShootPositionAndShoot(IntakePivot intakePivot, IntakeRollers intakeRollers){
        return new ParallelCommandGroup(
            new IntakeShooterPosition(intakePivot, IntakeShooterPositions.AUTON_SHOOT),
            new IntakeRollersCommand(intakeRollers, -1, 0)
        );
    }

    /**
     * Creates a command to move the intake pivot to the intake position and activate the rollers to intake.
     * @param intakePivot The intake pivot subsystem.
     * @param intakeRollers The intake rollers subsystem.
     * @return A command that moves the intake pivot and activates the rollers for intaking.
     */
    public Command goToIntakePositionAndIntake(IntakePivot intakePivot, IntakeRollers intakeRollers){
        return new ParallelCommandGroup(
            new IntakeShooterPosition(intakePivot, IntakeShooterPositions.AUTON_INTAKE),
            new IntakeRollersCommand(intakeRollers, 1, -1)
        );
    }

    /**
     * Creates a command to move the intake pivot to the auton_shoot position and activate the rollers for shooting.
     * @param intakePivot The intake pivot subsystem.
     * @param intakeRollers The intake rollers subsystem.
     * @return A command that moves the intake pivot to the auton_shoot position and activates the rollers for shooting.
     */
    public Command goToAutonShootPositionAndShoot(IntakePivot intakePivot, IntakeRollers intakeRollers){
        return new ParallelCommandGroup(
            new IntakeShooterPosition(intakePivot, IntakeShooterPositions.AUTON_SHOOT),
            new IntakeRollersCommand(intakeRollers, -1, 0)
        );
    }
}
