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
    private PIDController p;
    private double scaledRotateValue;

    public driveAlignVisionCommand(Vision vision, SwerveDrive swerveDrive){
        this.vision = vision;
        horizontalDifference = vision.getHorizontalDiff();
        this.m_SwerveDrive = swerveDrive;
        this.vision = vision;
        addRequirements(vision, swerveDrive);
    }
    
    @Override
    public void execute(){
        isAprilTagFound = vision.found();
        horizontalDifference = vision.getHorizontalDiff();
        if ((isAprilTagFound)){
            p = new PIDController(10, 0, 0);
            scaledRotateValue = p.calculate(-1*horizontalDifference);
            // scaledRotateValue = Math.min(-1.0, scaledRotateValue);
            // scaledRotateValue = Math.max(1.0, scaledRotateValue);
            System.out.println(scaledRotateValue);
            m_SwerveDrive.drive(0,0, scaledRotateValue, false, false);
        }
        else{
            m_SwerveDrive.drive(0,0,0,false,false);
        }
        //System.out.println(isAprilTagFound);

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}