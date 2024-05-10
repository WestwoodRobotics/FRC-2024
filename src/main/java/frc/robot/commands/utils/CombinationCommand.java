package frc.robot.commands.utils;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that combines multiple commands to be executed in parallel or sequence.
 * This command allows for the combination of multiple commands, which can be useful
 * for coordinating actions between different subsystems or for complex operations
 * that require multiple steps.
 */
public class CombinationCommand extends Command{

    /**
     * Constructs a CombinationCommand with the given commands.
     * @param commands The commands to be combined.
     */
    public CombinationCommand (Command... commands){
        // The implementation for adding the commands to a command group or similar structure goes here.
    }
    
}
