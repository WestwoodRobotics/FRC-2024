package frc.robot.subsystems.utils.Position_Enums;

/**
 * Enum representing various positions for the intake shooter mechanism.
 * These positions correspond to specific target states the intake shooter can be in.
 */
public enum IntakeShooterPositions {
    HOME, // Default position, usually the stowed position
    INTAKE, // Position for intaking balls
    SHOOT_NEAR_SPEAKER, // Position for shooting near the speaker
    SHOOT_NEAR_SPEAKER_FACING_FORWARDS, // Position for shooting near the speaker while facing forwards
    SHOOT_FAR_SPEAKER, // Position for shooting far from the speaker
    PODIUM_SHOT, // Position for shooting from the podium
    AUTON_SHOOT, // Position for shooting during autonomous mode
    AUTON_INTAKE, // Position for intaking during autonomous mode
    AMP, // Position for shooting at the amplifier target
    MANUAL // Position for manual control, not a preset position
}
