package frc.robot.commands.swerve;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class DriveAlignAndRangeVisionCommand extends Command {

    private Vision vision;
    private SwerveDrive m_SwerveDrive;
    private boolean isAprilTagFound;
    private double horizontalDifference;
    private double verticalDifference;
    private double distance;
    private PIDController p;
    private double scaledRotateValue;
    private double scaledForwardValue;
    private double scaledStrafeValue;
    private double rotateDistance;
    private double kP = .035; // constant of proportionality for aiming
    private double kP_range = .05; // constant of proportionality for ranging
    private double kP_strafe = .1; // constant of proportionality for strafing
    private double desiredDistance = 1.0; // desired distance from the April tag

    public DriveAlignAndRangeVisionCommand(Vision vision, SwerveDrive swerveDrive){
        this.vision = vision;
        this.m_SwerveDrive = swerveDrive;
        addRequirements(vision, swerveDrive);
    }
    
    @Override
    /*
     * From a top-down perspective
     */
    public void execute(){
        isAprilTagFound = vision.found();
        if (isAprilTagFound){
            double[] aprilTagTransform = vision.getAprilTagTransform();
            rotateDistance = vision.getHorizontalDiff();

            horizontalDifference = aprilTagTransform[1];
            verticalDifference = aprilTagTransform[0];

            // Aiming
            p = new PIDController(kP, 0, 0);
            double pidOutput = p.calculate(rotateDistance);
            scaledRotateValue = MathUtil.clamp(pidOutput, -1, 1); // clamp the value between -1 and 1

            // Ranging
            p = new PIDController(kP_range, 0, 0);
            pidOutput = p.calculate(distance - desiredDistance);
            scaledForwardValue = MathUtil.clamp(pidOutput, -1, 1); // clamp the value between -1 and 1

            // Strafing
            // p = new PIDController(kP_strafe, 0, 0);
            // pidOutput = p.calculate(horizontalDifference);
            // scaledStrafeValue = MathUtil.clamp(pidOutput, -1, 1); // clamp the value between -1 and 1

            //m_SwerveDrive.drive(scaledForwardValue, 0, scaledRotateValue, true, false);
        }
        else{
            m_SwerveDrive.drive(0,0,0,false,false);
        }
        double[] x = vision.getAprilTagTransform();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}