package frc.robot.subsystems.intakeShooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

public class IntakeShooter extends SubsystemBase {

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
        upperRollerPIDController = new PIDController(IntakeShooterConstants.kUpperRollerP, 
        IntakeShooterConstants.kUpperRollerI, IntakeShooterConstants.kUpperRollerD);
        lowerRollerPIDController = new PIDController(IntakeShooterConstants.kLowerRollerP, 
        IntakeShooterConstants.kLowerRollerI, IntakeShooterConstants.kLowerRollerD);
        pivotPIDController = new PIDController(IntakeShooterConstants.kPivotP, 
        IntakeShooterConstants.kPivotI, IntakeShooterConstants.kPivotD);
        intakeShooterPosition = IntakeShooterPositions.STOW;
    }

    public void setRollerPower(double power){
        upperRollerMotor.set(power);
        lowerRollerMotor.set(-power);
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

    public void setToPosition(IntakeShooterPositions position) {
        if (intakeShooterPosition != position || intakeShooterPosition == IntakeShooterPositions.MANUAL) {
            double setPoint;
            switch (position) {
                case STOW:
                    setPoint = IntakeShooterConstants.kStowPivotPosition;
                    break;
                case INTAKE:
                    setPoint = IntakeShooterConstants.kIntakePivotPosition;
                    break;
                case SHOOT_NEAR_SPEAKER:
                    setPoint = IntakeShooterConstants.kShootNearSpeakerPivotPosition;
                    break;
                case SHOOT_FAR_SPEAKER:
                    setPoint = IntakeShooterConstants.kShootFarSpeakerPivotPosition;
                    break;
                case HANDOFF:
                    setPoint = IntakeShooterConstants.kHandoffPivotPosition;
                    break;
                case MANUAL:
                    setPoint = pivotMotor.getEncoder().getPosition();
                    break;
                default:
                    //Should never happen, because the only way to call this method is with a valid position
                    throw new IllegalArgumentException("Invalid position: " + position);
            }

            pivotPIDController.setSetpoint(setPoint);
            pivotMotor.set(pivotPIDController.calculate(pivotMotor.getEncoder().getPosition()));
            intakeShooterPosition = position;
        }
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

    @Override
    public void periodic(){
        SmartDashboard.putString ("Intake Shooter State" , intakeShooterPosition.toString()); 
        SmartDashboard.putNumber("Pivot Position", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Upper Roller RPM", upperRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Lower Roller RPM", lowerRollerMotor.getEncoder().getVelocity());
    }

}
