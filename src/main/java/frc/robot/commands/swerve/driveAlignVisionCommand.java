package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

/**
 * Command for aligning the robot with a vision target using a PID controller.
 * This command uses vision data to rotate the robot to align with a target.
 */
public class driveAlignVisionCommand extends Command {

    private Vision vision;
    private SwerveDrive m_SwerveDrive;
    private boolean isAprilTagFound;
    private double horizontalDifference;
    private PIDController p;
    private double scaledRotateValue;

    /**
     * Constructs a new driveAlignVisionCommand.
     * 
     * @param vision The vision subsystem used to detect targets.
     * @param swerveDrive The swerve drive subsystem used to control the robot's movement.
     */
    public driveAlignVisionCommand(Vision vision, SwerveDrive swerveDrive){
        this.vision = vision;
        horizontalDifference = vision.getHorizontalDiff();
        this.m_SwerveDrive = swerveDrive;
        this.vision = vision;
        addRequirements(vision, swerveDrive);
    }
    
    /**
     * Executes the command to align the robot with the vision target.
     */
    @Override
    public void execute(){
        isAprilTagFound = vision.found();
        horizontalDifference = vision.getHorizontalDiff();
        if (isAprilTagFound){
            p = new PIDController(0.0325, 0, 0);
            double pidOutput = p.calculate(horizontalDifference);
            scaledRotateValue = MathUtil.clamp(pidOutput, -1, 1); // clamp the value between -1 and 1
            System.out.println(scaledRotateValue);
            m_SwerveDrive.drive(0,0, scaledRotateValue, false, false);
        }
        else{
            m_SwerveDrive.drive(0,0,Math.copySign(0.2, scaledRotateValue),false,false);
        }
    }

    /**
     * Determines if the command is finished.
     * 
     * @return false to keep the command running.
     */
    @Override
    public boolean isFinished(){
        return false;
    }
}
