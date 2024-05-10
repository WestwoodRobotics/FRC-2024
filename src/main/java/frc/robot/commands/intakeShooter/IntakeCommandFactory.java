package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intakeShooter.IntakePivot;
import frc.robot.subsystems.intakeShooter.IntakeRollers;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LimitSwitch;

/**
 * Factory class for creating intake and shooter related commands.
 * This class provides methods to create commands that combine actions on the intake pivot and rollers subsystems.
 */
public class IntakeCommandFactory {
    
    /**
     * Creates a command to move the intake pivot to the shooting position and activate the rollers for shooting.
     * @param intakePivotSubsystem The intake pivot subsystem
     * @param intakeRollersSubsystem The intake rollers subsystem
     * @param limitSwitchSensor The limit switch sensor to detect position
     * @return A command that moves the intake pivot to the shooting position and activates the rollers
     */
    public Command goToShootPositionAndShoot(IntakePivot intakePivotSubsystem, IntakeRollers intakeRollersSubsystem, LimitSwitch limitSwitchSensor){
        return new ParallelCommandGroup(
            new IntakeShooterPosition(intakePivotSubsystem, IntakeShooterPositions.AUTON_SHOOT, limitSwitchSensor),
            new IntakeRollersCommand(intakeRollersSubsystem, -1, 0)
        );
    }

    /**
     * Creates a command to move the intake pivot to the intake position and activate the rollers for intaking.
     * @param intakePivotSubsystem The intake pivot subsystem
     * @param intakeRollersSubsystem The intake rollers subsystem
     * @param limitSwitchSensor The limit switch sensor to detect position
     * @return A command that moves the intake pivot to the intake position and activates the rollers
     */
    public Command goToIntakePositionAndIntake(IntakePivot intakePivotSubsystem, IntakeRollers intakeRollersSubsystem, LimitSwitch limitSwitchSensor){
        return new ParallelCommandGroup(
            new IntakeShooterPosition(intakePivotSubsystem, IntakeShooterPositions.AUTON_INTAKE, limitSwitchSensor),
            new IntakeRollersCommand(intakeRollersSubsystem, 1, -1)
        );
    }
}
