package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class driveAlignVisionCommand extends Command {

    private Vision vision;
    private SwerveDrive m_SwerveDrive;

    private boolean isAprilTagFound;
    private double horizontalDifference;
    PIDController p;
    double scaledRotateValue;

    public driveAlignVisionCommand(Vision vision, SwerveDrive swerveDrive){
        this.vision = vision;
        horizontalDifference = vision.getHorizontalDiff();
    }
    
    @Override
    public void execute(){
    
        if ((horizontalDifference > 1) && (isAprilTagFound)){
            p = new PIDController(1, 0, 0);
            scaledRotateValue = p.calculate(horizontalDifference);
            m_SwerveDrive.drive(0,0,scaledRotateValue, false, false);
        }
    }

    @Override
    public boolean isFinished(){
        return (isAprilTagFound ? (horizontalDifference < 1) : (false));
    }
}