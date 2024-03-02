package frc.robot.subsystems.intakeShooter;

import com.revrobotics.CANSparkMax;

import java.util.HashMap;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
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
    HashMap<IntakeShooterPositions, Double> pivotPositionValues = new HashMap<>();

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

        
        pivotPositionValues.put(IntakeShooterPositions.INTAKE, IntakeShooterConstants.kIntakePivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.STOW, IntakeShooterConstants.kIntakePivotPosition);

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

    public boolean setToPosition(IntakeShooterPositions position) {
        double setPoint = 0;
            
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
            return (pivotMotor.getEncoder().getPosition() == setPoint);
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

    public boolean setPivotPositionNOPID (IntakeShooterPositions positions){
        double positionValue = pivotPositionValues.get(positions);
        double currentPosition = pivotMotor.getEncoder().getPosition();
        
        if (positionValue < currentPosition){
            pivotPIDController.setSetpoint(positionValue);
            pivotMotor.set(pivotPIDController.calculate(pivotMotor.getEncoder().getPosition())/4);
            System.out.println("Pivot Going 1");
            return false;
        } 
        if(positionValue > currentPosition){
            pivotPIDController.setSetpoint(positionValue);
            pivotMotor.set(pivotPIDController.calculate(pivotMotor.getEncoder().getPosition())/4);
            System.out.println("Pivot Going 2");
        }
        if (positionValue == currentPosition){
            pivotMotor.set(0);
            //elevatorPivotPosition = positions;
            System.out.println("Pivot Reached");
            return true;
        }
        return false;
    }

    @Override
    public void periodic(){
        SmartDashboard.putString ("Intake Shooter State" , intakeShooterPosition.toString()); 
        SmartDashboard.putNumber("Pivot Position", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Upper Roller RPM", upperRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Lower Roller RPM", lowerRollerMotor.getEncoder().getVelocity());
    }

}
