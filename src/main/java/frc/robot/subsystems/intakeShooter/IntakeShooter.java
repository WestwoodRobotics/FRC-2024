package frc.robot.subsystems.intakeShooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

public class IntakeShooter {

    private CANSparkMax upperRollerMotor;
    private CANSparkMax lowerRollerMotor;

    private PIDController upperRollerPIDController;
    private PIDController lowerRollerPIDController;


    private CANSparkMax stowMotor;

    private CANSparkMax pivotMotor;
    private PIDController pivotPIDController;

    private IntakeShooterPositions intakeShooterPosition;

    public IntakeShooter(){
        upperRollerMotor = new CANSparkMax(IntakeShooterConstants.kUpperMotorPort, MotorType.kBrushless);
        lowerRollerMotor = new CANSparkMax(IntakeShooterConstants.kLowerMotorPort, MotorType.kBrushless);
        stowMotor = new CANSparkMax(IntakeShooterConstants.kStowMotorPort, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(IntakeShooterConstants.kPivotMotorPort, MotorType.kBrushless);
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        upperRollerPIDController = new PIDController(IntakeShooterConstants.kUpperRollerP, IntakeShooterConstants.kUpperRollerI, IntakeShooterConstants.kUpperRollerD);
        lowerRollerPIDController = new PIDController(IntakeShooterConstants.kLowerRollerP, IntakeShooterConstants.kLowerRollerI, IntakeShooterConstants.kLowerRollerD);
        pivotPIDController = new PIDController(IntakeShooterConstants.kPivotP, IntakeShooterConstants.kPivotI, IntakeShooterConstants.kPivotD);
        intakeShooterPosition = IntakeShooterPositions.STOW;
    }

    public void setRollerPower(double power){
        upperRollerMotor.set(power);
        lowerRollerMotor.set(power);
    }

    public void setRollerRPM (double upperRollerRPM, double lowerRollerRPM){
        upperRollerPIDController.setSetpoint(upperRollerRPM);
        lowerRollerPIDController.setSetpoint(lowerRollerRPM);
        upperRollerMotor.set(upperRollerPIDController.calculate(upperRollerMotor.getEncoder().getVelocity() + IntakeShooterConstants.kUpperRollerFF));
        lowerRollerMotor.set(lowerRollerPIDController.calculate(lowerRollerMotor.getEncoder().getVelocity() + IntakeShooterConstants.kLowerRollerFF));
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
        pivotMotor.set(power);
        intakeShooterPosition = IntakeShooterPositions.MANUAL;
    }

    public void setStowPosition(){
        if ((intakeShooterPosition != IntakeShooterPositions.STOW) || intakeShooterPosition == IntakeShooterPositions.MANUAL) {
            pivotPIDController.setSetpoint(IntakeShooterConstants.kStowPivotPosition);
            pivotMotor.set(pivotPIDController.calculate(pivotMotor.getEncoder().getPosition()));
            intakeShooterPosition = IntakeShooterPositions.STOW;
        }
    }

    public void setIntakePosition(){
        if ((intakeShooterPosition != IntakeShooterPositions.INTAKE) || intakeShooterPosition == IntakeShooterPositions.MANUAL) {
            pivotPIDController.setSetpoint(IntakeShooterConstants.kIntakePivotPosition);
            pivotMotor.set(pivotPIDController.calculate(pivotMotor.getEncoder().getPosition()));
            intakeShooterPosition = IntakeShooterPositions.INTAKE;
        }
    }

    public void setShootNearSpeakerPosition(){
        if ((intakeShooterPosition != IntakeShooterPositions.SHOOT_NEAR_SPEAKER) || intakeShooterPosition == IntakeShooterPositions.MANUAL) {
            pivotPIDController.setSetpoint(IntakeShooterConstants.kShootNearSpeakerPivotPosition);
            pivotMotor.set(pivotPIDController.calculate(pivotMotor.getEncoder().getPosition()));
            intakeShooterPosition = IntakeShooterPositions.SHOOT_NEAR_SPEAKER;
        }
    }

    public void setShootFarSpeakerPosition(){
        if ((intakeShooterPosition != IntakeShooterPositions.SHOOT_FAR_SPEAKER) || intakeShooterPosition == IntakeShooterPositions.MANUAL) {
            pivotPIDController.setSetpoint(IntakeShooterConstants.kShootFarSpeakerPivotPosition);
            pivotMotor.set(pivotPIDController.calculate(pivotMotor.getEncoder().getPosition()));
            intakeShooterPosition = IntakeShooterPositions.SHOOT_FAR_SPEAKER;
        }
    }

}
