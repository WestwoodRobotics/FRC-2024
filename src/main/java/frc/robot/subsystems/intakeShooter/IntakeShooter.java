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

public class IntakeShooter extends SubsystemBase {

    private CANSparkMax upperRollerMotor;
    private CANSparkMax lowerRollerMotor;

    private PIDController upperRollerPIDController;
    private PIDController lowerRollerPIDController;


    private CANSparkMax stowMotor;

    public CANSparkMax pivotMotor;
    private PIDController pivotPIDController;

    private IntakeShooterPositions intakeShooterPosition;
    private HashMap<IntakeShooterPositions, Double> pivotPositionValues = new HashMap<>();
    private boolean isRollerPIDControl;
    private boolean isPivotPIDControl;
    private double calculatedPivotPIDValue;



    private LimitSwitch l;
    public IntakeShooter(LimitSwitch l){
        this.l = l;
        upperRollerMotor = new CANSparkMax(IntakeShooterConstants.kUpperMotorPort, MotorType.kBrushless);
        lowerRollerMotor = new CANSparkMax(IntakeShooterConstants.kLowerMotorPort, MotorType.kBrushless);
        stowMotor = new CANSparkMax(IntakeShooterConstants.kStowMotorPort, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(IntakeShooterConstants.kPivotMotorPort, MotorType.kBrushless);
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        
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


    }

    public void setRollerPower(double power)
    {
        isRollerPIDControl = false;
        upperRollerMotor.set(power);
        lowerRollerMotor.set(-power);
    }

    public boolean getPivotLimitReached()
    {
        return l.getStatus();
    }

    public void setRollerRPM (double upperRollerRPM, double lowerRollerRPM)
    {
        isRollerPIDControl = true;
        upperRollerPIDController.setSetpoint(upperRollerRPM);
        lowerRollerPIDController.setSetpoint(lowerRollerRPM);
        // upperRollerMotor.set(upperRollerPIDController.calculate(upperRollerMotor.getEncoder().getVelocity() + IntakeShooterConstants.kUpperRollerFF));
        // lowerRollerMotor.set(lowerRollerPIDController.calculate(lowerRollerMotor.getEncoder().getVelocity() + IntakeShooterConstants.kLowerRollerFF));
    }

    public double getUpperRPM(){
        return upperRollerMotor.getEncoder().getVelocity();
    }

    public double getLowerRPM(){
        return lowerRollerMotor.getEncoder().getVelocity();
    }
    
    public void setStowPower(double power){
        stowMotor.set(power);
    }

    public void setPivotPower(double power){
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

        if (l.getStatus() && ((position == IntakeShooterPositions.HOME))){
            return true;
        }
        double setPoint = pivotPositionValues.get(position);     
        pivotPIDController.setSetpoint(setPoint);
        calculatedPivotPIDValue = pivotPIDController.calculate(pivotMotor.getEncoder().getPosition());
        //if (calculatedPivotPIDValue > 0) {
            pivotMotor.set(calculatedPivotPIDValue);
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
        stopRollerMotor();
        stopStowMotor();
    }

    public void stopAllIntakeShooterRollers(){
        upperRollerMotor.set(0);
        lowerRollerMotor.set(0);
        stowMotor.set(0);
    }
    public void stopPivotMotor(){
        pivotMotor.set(0);
    }

    public void stopRollerMotor(){
        upperRollerMotor.set(0);
        lowerRollerMotor.set(0);
    }

    public void stopStowMotor(){
        stowMotor.set(0);
    }

    public IntakeShooterPositions getState() {
        return intakeShooterPosition;
    }

    public void resetEncoder(){
        pivotMotor.getEncoder().setPosition(0);
    }

    public boolean setPivotPosition (IntakeShooterPositions positions){
        double positionValue = pivotPositionValues.get(positions);
        double currentPosition = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        if (Math.abs(positionValue - currentPosition) <= 0.05){
            pivotMotor.set(0);
            //elevatorPivotPosition = positions;
            System.out.println("Pivot Reached Setpoint");
            return true;
        }
        else if (positionValue < currentPosition){
            pivotPIDController.setSetpoint(positionValue);
            pivotMotor.set(pivotPIDController.calculate(currentPosition)/4);
            System.out.println("Pivot Below Setpoint");
            return false;
        } 
        else if(positionValue > currentPosition){
            pivotPIDController.setSetpoint(positionValue);
            pivotMotor.set(pivotPIDController.calculate(currentPosition)/4);
            System.out.println("Pivot Above Setpoint");
        }
        return false;
    }

    public void setPositionState(IntakeShooterPositions position){
        intakeShooterPosition = position;
    }

    @Override
    public void periodic(){
        SmartDashboard.putString ("Intake Shooter State" , intakeShooterPosition.toString()); 
        SmartDashboard.putNumber("Pivot Position", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Upper Roller RPM", upperRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Lower Roller RPM", lowerRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Limit", this.getPivotLimitReached());

        if (isRollerPIDControl){
            upperRollerMotor.set(-1*lowerRollerPIDController.calculate(upperRollerMotor.getEncoder().getVelocity())+IntakeShooterConstants.kUpperRollerFF);
            lowerRollerMotor.set(lowerRollerPIDController.calculate(lowerRollerMotor.getEncoder().getVelocity())+IntakeShooterConstants.kLowerRollerFF);
        }




    }

}

