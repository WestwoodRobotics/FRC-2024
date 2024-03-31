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

public class IntakePivot extends SubsystemBase {
    public CANSparkMax pivotMotor;
    private PIDController pivotPIDController;

    private IntakeShooterPositions intakeShooterPosition;
    private HashMap<IntakeShooterPositions, Double> pivotPositionValues = new HashMap<>();

    private boolean isPivotPIDControl;
    private double calculatedPivotPIDValue;



    private LimitSwitch l;

    public IntakePivot(LimitSwitch l){
        this.l = l;
        pivotMotor = new CANSparkMax(IntakeShooterConstants.kPivotMotorPort, MotorType.kBrushless);
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        

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


    }


    public boolean getPivotLimitReached()
    {
        return l.getStatus();
    }


    public void setPivotPower(double power){
        isPivotPIDControl = false;
        if (this.getPivotLimitReached() && power < 0){
            pivotMotor.set(0);
            this.resetEncoder();
        }
        else{
            pivotMotor.set(power);
            intakeShooterPosition = IntakeShooterPositions.MANUAL;
        }
    }


    // public boolean setToPosition(IntakeShooterPositions position) {
    //     if (l.getStatus() && ((position == IntakeShooterPositions.HOME))){
    //         return true;
    //     }
        
    //     double setPoint = pivotPositionValues.get(position);     
    //     pivotPIDController.setSetpoint(setPoint);
    //     pivotMotor.set(pivotPIDController.calculate(pivotMotor.getEncoder().getPosition()));
    //     intakeShooterPosition = position;
    //     //return Math.abs(pivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() - setPoint) <= 0.05;
    //     return Math.abs(pivotMotor.getEncoder().getPosition() - setPoint) <= 1;
    // }

    public boolean setToPosition(IntakeShooterPositions position) {
        isPivotPIDControl = true;
        if (l.getStatus() && ((position == IntakeShooterPositions.HOME))){
            return true;
        }
        double setPoint = pivotPositionValues.get(position);     
        pivotPIDController.setSetpoint(setPoint);

        //}
        //else {
        //    pivotMotor.set(0);
        //} 
        intakeShooterPosition = position;
        //return Math.abs(pivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() - setPoint) <= 0.05;
        return (Math.abs(pivotMotor.getEncoder().getPosition() - setPoint) <= 1);
    }   



    public void stopAllMotors(){
        stopPivotMotor();
    }
    public void stopPivotMotor(){
        pivotMotor.set(0);
    }

    public IntakeShooterPositions getState() {
        return intakeShooterPosition;
    }

    public void resetEncoder(){
        pivotMotor.getEncoder().setPosition(0);
    }

    public void setPositionState(IntakeShooterPositions position){
        intakeShooterPosition = position;
    }

    @Override
    public void periodic(){
        SmartDashboard.putString ("Intake Shooter State" , intakeShooterPosition.toString()); 
        SmartDashboard.putNumber("Pivot Position", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Limit", this.getPivotLimitReached());

        if (isPivotPIDControl = true){
            calculatedPivotPIDValue = pivotPIDController.calculate(pivotMotor.getEncoder().getPosition());
            pivotMotor.set(calculatedPivotPIDValue);
        }

    }

}

