package frc.robot.subsystems.intakeShooter;
import com.revrobotics.CANSparkMax;
import java.util.HashMap;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

/**
 * The IntakePivot subsystem controls the pivot mechanism of the intake shooter.
 * It uses a CANSparkMax motor for movement and a PIDController for precise positioning.
 */
public class IntakePivot extends SubsystemBase {
    private CANSparkMax intakePivotMotorController;
    private PIDController intakePivotPositionPIDController;


    private HashMap<IntakeShooterPositions, Double> intakePivotPositionTargetValues = new HashMap<>();


    
    private double calculatedIntakePivotPIDValue;

    /**
     * Constructor for the IntakePivot subsystem.
     */
    public IntakePivot(){
        intakePivotMotorController = new CANSparkMax(IntakeShooterConstants.kPivotMotorPort, MotorType.kBrushless);
        intakePivotMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakePivotPositionPIDController.setTolerance(0.2);

        intakePivotPositionPIDController = new PIDController(IntakeShooterConstants.kPivotP, 
        IntakeShooterConstants.kPivotI, IntakeShooterConstants.kPivotD);
        
        
        intakePivotPositionTargetValues.put(IntakeShooterPositions.INTAKE, IntakeShooterConstants.kIntakePivotPosition);
        intakePivotPositionTargetValues.put(IntakeShooterPositions.HOME, IntakeShooterConstants.kHomePivotPosition);
        intakePivotPositionTargetValues.put(IntakeShooterPositions.SHOOT_NEAR_SPEAKER, IntakeShooterConstants.kShootNearSpeakerPivotPosition);
        intakePivotPositionTargetValues.put(IntakeShooterPositions.SHOOT_FAR_SPEAKER, IntakeShooterConstants.kShootFarSpeakerPivotPosition);
        intakePivotPositionTargetValues.put(IntakeShooterPositions.AUTON_SHOOT, IntakeShooterConstants.kShootNearSpeakerAutonPivotPosition);
        intakePivotPositionTargetValues.put(IntakeShooterPositions.AUTON_INTAKE, IntakeShooterConstants.kAutoIntakePivotPosition);
        intakePivotPositionTargetValues.put(IntakeShooterPositions.AMP, IntakeShooterConstants.kShootPivotAmp);
        intakePivotPositionTargetValues.put(IntakeShooterPositions.SHOOT_NEAR_SPEAKER_FACING_FORWARDS, IntakeShooterConstants.kShootNearSpeakerFacingForwardsPivotPosition);
        intakePivotPositionTargetValues.put(IntakeShooterPositions.PODIUM_SHOT, IntakeShooterConstants.kShootPodiumShot);
    }

    /**
     * Sets the power for the pivot motor.
     * 
     * @param power The power level to set for the pivot motor.
     */
    public void setPivotPower(double power){
        intakePivotMotorController.set(power);
    }

    /**
     * Moves the pivot to a specified position using PID control.
     * 
     * @param position The target position for the pivot.
     * @return True if the pivot has reached the target position, false otherwise.
     */
    public void setToPosition(IntakeShooterPositions position) {
        double setPoint = intakePivotPositionTargetValues.get(position);     
        intakePivotPositionPIDController.setSetpoint(setPoint);
    }   

    public void setToPosition(double position) {
        intakePivotPositionPIDController.setSetpoint(position);
    }   

    /**
     * Stops all motors in the IntakePivot subsystem.
     */
    public void stopAllMotors(){
        stopPivotMotor();
    }

    /**
     * Stops the pivot motor.
     */
    public void stopPivotMotor(){
        intakePivotMotorController.set(0);
    }

    /**
     * Resets the encoder for the pivot motor.
     */
    public void resetEncoder(){
        intakePivotMotorController.getEncoder().setPosition(0);
    }

    /**
     * Sets the state of the IntakePivot subsystem.
     * 
     * @param position The new state for the IntakePivot subsystem.
     */
    public void setPositionState(IntakeShooterPositions position){

    }

    public boolean isPivotAtTargetPIDPosition(){
        return intakePivotPositionPIDController.atSetpoint();
    }

    public CANSparkMax getIntakePivotMotorController(){
        return intakePivotMotorController;
    }

    public PIDController getIntakePivotPositionPIDController(){
        return intakePivotPositionPIDController;
    }

    public double getCalculatedIntakePivotPIDValue(){
        return intakePivotPositionPIDController.calculate(intakePivotMotorController.getEncoder().getPosition());
    }



    /**
     * Periodically updates the subsystem's state and handles PID control for the pivot.
     */
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pivot Position", intakePivotMotorController.getEncoder().getPosition());
    }
}
