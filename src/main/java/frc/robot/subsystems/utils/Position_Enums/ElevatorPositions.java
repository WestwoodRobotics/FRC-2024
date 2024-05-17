package frc.robot.subsystems.utils.Position_Enums;

/**
 * Enum representing various positions for the elevator mechanism.
 * These positions correspond to specific target heights or states the elevator can be in.
 */
public enum ElevatorPositions {
    PODIUM, // Position for reaching the podium height
    AMP, // Position for reaching the amplifier height
    HOME, // Default position, usually the lowest point
    SOURCE, // Position for reaching the source height
    MANUAL, // Position for manual control, not a preset height
    AUTO_SHOOT, // Position for automatically shooting
    HANDOFF // Position for handing off game pieces
}
