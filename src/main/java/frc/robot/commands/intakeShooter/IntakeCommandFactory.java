package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intakeShooter.IntakePivot;
import frc.robot.subsystems.intakeShooter.IntakeRollers;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LimitSwitch;



public class IntakeCommandFactory {
    
    public Command goToShootPositionAndShoot(IntakePivot intakePivot, IntakeRollers intakeRollers, LimitSwitch l){
        return new ParallelCommandGroup(
            new IntakeShooterPosition(intakePivot, IntakeShooterPositions.AUTON_SHOOT, l),
            new IntakeRollersCommand(intakeRollers, -1, 0)
        );
    }

    public Command goToIntakePositionAndIntake(IntakePivot intakePivot, IntakeRollers intakeRollers, LimitSwitch l){
        return new ParallelCommandGroup(
            new IntakeShooterPosition(intakePivot, IntakeShooterPositions.AUTON_INTAKE, l),
            new IntakeRollersCommand(intakeRollers, 1, -1)
        );
    }

    





}