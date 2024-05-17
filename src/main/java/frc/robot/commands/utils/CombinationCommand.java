package frc.robot.commands.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * A command that combines multiple commands into a sequential execution.
 * This command will execute each command in the order they are added.
 */
public class CombinationCommand extends SequentialCommandGroup {

    /**
     * Constructs a new CombinationCommand instance.
     * 
     * @param commands The commands to be combined and executed sequentially.
     */
    public CombinationCommand(Command... commands) {
        super(commands);
    }
}
