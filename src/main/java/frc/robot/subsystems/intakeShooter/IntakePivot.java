package frc.robot.subsystems.intakeShooter;
import com.revrobotics.CANSparkMax;
import java.util.HashMap;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LimitSwitch;

/**
 * The IntakePivot subsystem controls the pivot mechanism of the intake shooter.
 * It uses a CANSparkMax motor for movement and a PIDController for precise positioning.
 * A LimitSwitch is used to detect the home position.
 */
public class IntakePivot extends SubsystemBase {
    public CANSparkMax intakePivotMotorController;
    private PIDController intakePivotPositionPIDController;

    private IntakeShooterPositions intakePivotPosition;
    private HashMap<IntakeShooterPositions, Double> intakePivotPositionTargetValues = new HashMap<>();

    private boolean isIntakePivotPIDControlEnabled;
    
    private double calculatedIntakePivotPIDValue;

    private LimitSwitch intakePivotLimitSwitch;

    /**
     * Constructor for the IntakePivot subsystem.
     * 
     * @param limitSwitch The limit switch used to determine if the pivot is at the home position.
     */
    public IntakePivot(LimitSwitch limitSwitch){
        this.intakePivotLimitSwitch = limitSwitch;
        intakePivotMotorController = new CANSparkMax(IntakeShooterConstants.kPivotMotorPort, MotorType.kBrushless);
        intakePivotMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        

        intakePivotPositionPIDController = new PIDController(IntakeShooterConstants.kPivotP, 
        IntakeShooterConstants.kPivotI, IntakeShooterConstants.kPivotD);

        intakePivotPosition = IntakeShooterPositions.HOME;

        
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
     * Checks if the pivot has reached the limit switch.
     * 
     * @return True if the limit switch is activated, indicating the pivot is at the home position.
     */
    public boolean getPivotLimitReached()
    {
        return intakePivotLimitSwitch.getStatus();
    }

    /**
     * Sets the power for the pivot motor.
     * 
     * @param power The power level to set for the pivot motor.
     */
    public void setPivotPower(double power){
        isIntakePivotPIDControlEnabled = false;
        if (this.getPivotLimitReached() && power < 0){
            intakePivotMotorController.set(0);
            this.resetEncoder();
        }
        else{
            intakePivotMotorController.set(power);
            intakePivotPosition = IntakeShooterPositions.MANUAL;
        }
    }

    /**
     * Moves the pivot to a specified position using PID control.
     * 
     * @param position The target position for the pivot.
     * @return True if the pivot has reached the target position, false otherwise.
     */
    public boolean setToPosition(IntakeShooterPositions position) {
        isIntakePivotPIDControlEnabled = true;
        if (intakePivotLimitSwitch.getStatus() && ((position == IntakeShooterPositions.HOME))){
            return true;
        }
        double setPoint = intakePivotPositionTargetValues.get(position);     
        intakePivotPositionPIDController.setSetpoint(setPoint);

        intakePivotPosition = position;
        return (Math.abs(intakePivotMotorController.getEncoder().getPosition() - setPoint) <= 1);
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
     * Gets the current state of the IntakePivot subsystem.
     * 
     * @return The current state of the IntakePivot subsystem.
     */
    public IntakeShooterPositions getState() {
        return intakePivotPosition;
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
        intakePivotPosition = position;
    }

    /**
     * Periodically updates the subsystem's state and handles PID control for the pivot.
     */
    @Override
    public void periodic(){
        SmartDashboard.putString ("Intake Shooter State" , intakePivotPosition.toString()); 
        SmartDashboard.putNumber("Pivot Position", intakePivotMotorController.getEncoder().getPosition());
        SmartDashboard.putBoolean("Limit", this.getPivotLimitReached());

        if (isIntakePivotPIDControlEnabled = true){
            intakePivotPositionPIDController.setTolerance(0.2);
            calculatedIntakePivotPIDValue = intakePivotPositionPIDController.calculate(intakePivotMotorController.getEncoder().getPosition());
            intakePivotMotorController.set(calculatedIntakePivotPIDValue);
        }
    }
}
