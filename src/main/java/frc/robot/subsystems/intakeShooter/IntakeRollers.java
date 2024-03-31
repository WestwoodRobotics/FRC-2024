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

public class IntakeRollers extends SubsystemBase {

    private CANSparkMax upperRollerMotor;
    private CANSparkMax lowerRollerMotor;

    private PIDController upperRollerPIDController;
    private PIDController lowerRollerPIDController;


    private CANSparkMax stowMotor;



    private IntakeShooterPositions intakeShooterPosition;
    private boolean isRollerPIDControl;



    private LimitSwitch l;

    public IntakeRollers(){

        upperRollerMotor = new CANSparkMax(IntakeShooterConstants.kUpperMotorPort, MotorType.kBrushless);
        lowerRollerMotor = new CANSparkMax(IntakeShooterConstants.kLowerMotorPort, MotorType.kBrushless);
        stowMotor = new CANSparkMax(IntakeShooterConstants.kStowMotorPort, MotorType.kBrushless);
        
        upperRollerPIDController = new PIDController(IntakeShooterConstants.kUpperRollerP, 
        IntakeShooterConstants.kUpperRollerI, IntakeShooterConstants.kUpperRollerD);
        lowerRollerPIDController = new PIDController(IntakeShooterConstants.kLowerRollerP, 
        IntakeShooterConstants.kLowerRollerI, IntakeShooterConstants.kLowerRollerD);
        intakeShooterPosition = IntakeShooterPositions.HOME;
    }

    public void setRollerPower(double power)
    {
        isRollerPIDControl = false;
        upperRollerMotor.set(power);
        lowerRollerMotor.set(-power);
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
        isRollerPIDControl = false;
        stowMotor.set(power);
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

    public void stopAllMotors(){
        stopRollerMotor();
        stopStowMotor();
    }

    public void stopAllIntakeShooterRollers(){
        upperRollerMotor.set(0);
        lowerRollerMotor.set(0);
        stowMotor.set(0);
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


    public void setPositionState(IntakeShooterPositions position){
        intakeShooterPosition = position;
    }

    @Override
    public void periodic(){
        SmartDashboard.putString ("Intake Shooter State" , intakeShooterPosition.toString()); 
        SmartDashboard.putNumber("Upper Roller RPM", upperRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Lower Roller RPM", lowerRollerMotor.getEncoder().getVelocity());

        if (isRollerPIDControl){
            upperRollerMotor.set(-1*lowerRollerPIDController.calculate(upperRollerMotor.getEncoder().getVelocity())+IntakeShooterConstants.kUpperRollerFF);
            lowerRollerMotor.set(lowerRollerPIDController.calculate(lowerRollerMotor.getEncoder().getVelocity())+IntakeShooterConstants.kLowerRollerFF);
        }




    }

}

