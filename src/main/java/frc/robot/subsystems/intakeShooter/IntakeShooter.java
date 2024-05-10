package frc.robot.subsystems.intakeShooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import java.util.HashMap;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.BeamBreak;
import frc.robot.subsystems.vision.LimitSwitch;

import com.revrobotics.jni.CANSparkMaxJNI;

/**
 * The IntakeShooter subsystem controls the intake and shooting mechanisms for balls.
 * It includes methods to control the motors for the rollers, stow, and pivot mechanisms,
 * as well as methods to set the position of the pivot mechanism.
 */
public class IntakeShooter extends SubsystemBase {

    private CANSparkMax upperIntakeRollerMotor; // Renamed for clarity
    private CANSparkMax lowerRollerMotor;

    private PIDController upperRollerPIDController;
    private PIDController lowerRollerPIDController;

    private CANSparkMax stowMotor;

    public CANSparkMax intakePivotMotor; // Renamed for clarity
    private PIDController pivotPIDController;

    private IntakeShooterPositions intakeShooterPosition;
    private HashMap<IntakeShooterPositions, Double> pivotPositionValues = new HashMap<>();
    private boolean isRollerPIDControl;
    private boolean isPivotPIDControl;
    private double calculatedPivotPIDValue;

    private LimitSwitch limitSwitch; // Renamed for clarity

    /**
     * Constructs an IntakeShooter subsystem with a limit switch for pivot position detection.
     * 
     * @param limitSwitch The limit switch used to detect the home position of the pivot.
     */
    public IntakeShooter(LimitSwitch limitSwitch){
        this.limitSwitch = limitSwitch;
        upperIntakeRollerMotor = new CANSparkMax(IntakeShooterConstants.kUpperMotorPort, MotorType.kBrushless);
        lowerRollerMotor = new CANSparkMax(IntakeShooterConstants.kLowerMotorPort, MotorType.kBrushless);
        stowMotor = new CANSparkMax(IntakeShooterConstants.kStowMotorPort, MotorType.kBrushless);
        intakePivotMotor = new CANSparkMax(IntakeShooterConstants.kPivotMotorPort, MotorType.kBrushless);
        intakePivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        
        upperRollerPIDController = new PIDController(IntakeShooterConstants.kUpperRollerP, 
        IntakeShooterConstants.kUpperRollerI, IntakeShooterConstants.kUpperRollerD);
        lowerRollerPIDController = new PIDController(IntakeShooterConstants.kLowerRollerP, 
        IntakeShooterConstants.kLowerRollerI, IntakeShooterConstants.kLowerRollerD);
        pivotPIDController = new PIDController(IntakeShooterConstants.kPivotP, 
        IntakeShooterConstants.kPivotI, IntakeShooterConstants.kPivotD);
        intakeShooterPosition = IntakeShooterPositions.HOME;

        
        pivotPositionValues.put(IntakeShooterPositions.INTAKE, IntakeShooterConstants.kIntakePivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.HOME, IntakeShooterConstants.kHomePivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.SHOOT_NEAR_SPEAKER, IntakeShooterConstants.kShootNearSpeakerPivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.SHOOT_FAR_SPEAKER, IntakeShooterConstants.kShootFarSpeakerPivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.AUTON_SHOOT, IntakeShooterConstants.kShootNearSpeakerAutonPivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.AUTON_INTAKE, IntakeShooterConstants.kAutoIntakePivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.AMP, IntakeShooterConstants.kShootPivotAmp);
        pivotPositionValues.put(IntakeShooterPositions.SHOOT_NEAR_SPEAKER_FACING_FORWARDS, IntakeShooterConstants.kShootNearSpeakerFacingForwardsPivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.PODIUM_SHOT, IntakeShooterConstants.kShootPodiumShot);


    }

    /**
     * Sets the power for both the upper and lower intake roller motors.
     * @param power The power to set, where 1 is full forward and -1 is full reverse.
     */
    public void setRollerPower(double power)
    {
        isRollerPIDControl = false;
        upperIntakeRollerMotor.set(power);
        lowerRollerMotor.set(-power);
    }

    /**
     * Checks if the pivot has reached its limit switch indicating the home position.
     * 
     * @return True if the limit switch is activated, false otherwise.
     */
    public boolean getPivotLimitReached()
    {
        return limitSwitch.getStatus();
    }

    /**
     * Sets the RPM for both the upper and lower intake roller motors using PID control.
     * @param upperRollerRPM The target RPM for the upper roller motor.
     * @param lowerRollerRPM The target RPM for the lower roller motor.
     */
    public void setRollerRPM (double upperRollerRPM, double lowerRollerRPM)
    {
        isRollerPIDControl = true;
        upperRollerPIDController.setSetpoint(upperRollerRPM);
        lowerRollerPIDController.setSetpoint(lowerRollerRPM);
    }

    public double getUpperRPM(){
        return upperIntakeRollerMotor.getEncoder().getVelocity();
    }

    public double getLowerRPM(){
        return lowerRollerMotor.getEncoder().getVelocity();
    }
    
    /**
     * Sets the power for the stow motor.
     * @param power The power to set, where 1 is full forward and -1 is full reverse.
     */
    public void setStowPower(double power){
        stowMotor.set(power);
    }

    /**
     * Sets the power for the pivot motor.
     * 
     * @param power The power to set for the pivot motor, ranging from -1.0 to 1.0.
     */
    public void setPivotPower(double power){
        if (this.getPivotLimitReached() && power < 0){
            intakePivotMotor.set(0);
            this.resetEncoder();
        }
        else{
            intakePivotMotor.set(power);
            intakeShooterPosition = IntakeShooterPositions.MANUAL;
        }
    }

    public boolean setToPosition(IntakeShooterPositions position) {

        if (limitSwitch.getStatus() && ((position == IntakeShooterPositions.HOME))){
            return true;
        }
        double setPoint = pivotPositionValues.get(position);     
        pivotPIDController.setSetpoint(setPoint);
        calculatedPivotPIDValue = pivotPIDController.calculate(intakePivotMotor.getEncoder().getPosition());
        intakePivotMotor.set(calculatedPivotPIDValue);
        intakeShooterPosition = position;
        return (Math.abs(intakePivotMotor.getEncoder().getPosition() - setPoint) <= 1);
    }   

    /**
     * Stops all motors in the IntakeShooter subsystem.
     */
    public void stopAllMotors(){
        stopPivotMotor();
        stopRollerMotor();
        stopStowMotor();
    }

    /**
     * Stops all intake and shooter roller motors.
     */
    public void stopAllIntakeShooterRollers(){
        upperIntakeRollerMotor.set(0);
        lowerRollerMotor.set(0);
        stowMotor.set(0);
    }

    /**
     * Stops the pivot motor.
     */
    public void stopPivotMotor(){
        intakePivotMotor.set(0);
    }

    /**
     * Stops the roller motors.
     */
    public void stopRollerMotor(){
        upperIntakeRollerMotor.set(0);
        lowerRollerMotor.set(0);
    }

    /**
     * Stops the stow motor.
     */
    public void stopStowMotor(){
        stowMotor.set(0);
    }

    /**
     * Gets the current state of the IntakeShooter subsystem.
     * 
     * @return The current state of the IntakeShooter subsystem.
     */
    public IntakeShooterPositions getState() {
        return intakeShooterPosition;
    }

    /**
     * Resets the encoder for the pivot motor.
     */
    public void resetEncoder(){
        intakePivotMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putString ("Intake Shooter State" , intakeShooterPosition.toString()); 
        SmartDashboard.putNumber("Pivot Position", intakePivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Upper Roller RPM", upperIntakeRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Lower Roller RPM", lowerRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Limit", this.getPivotLimitReached());

        if (isRollerPIDControl){
            upperIntakeRollerMotor.set(-1*lowerRollerPIDController.calculate(upperIntakeRollerMotor.getEncoder().getVelocity())+IntakeShooterConstants.kUpperRollerFF);
            lowerRollerMotor.set(lowerRollerPIDController.calculate(lowerRollerMotor.getEncoder().getVelocity())+IntakeShooterConstants.kLowerRollerFF);
        }
    }
}
