package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
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
        if (isAprilTagFound){
            p = new PIDController(10, 0, 0);
            double pidOutput = p.calculate(horizontalDifference); // remove the -1 multiplier
            scaledRotateValue = MathUtil.clamp(pidOutput, -1, 1); // clamp the value between -1 and 1
            System.out.println(scaledRotateValue);
            m_SwerveDrive.drive(0,0, scaledRotateValue, false, false);
        }
        else{
            m_SwerveDrive.drive(0,0,0,false,false);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}