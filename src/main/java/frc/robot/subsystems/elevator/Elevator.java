package frc.robot.subsystems.elevator;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.utils.MotorControlGroup;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;

public class Elevator extends SubsystemBase{

    private CANSparkMax elevatorMotor1;
    private CANSparkMax elevatorMotor2;

    private PIDController elevatorPIDController;
    private PIDController elevatorPivotPIDController;

    private MotorControlGroup elevatorMotors;
    private CANSparkMax pivotMotor;
    private CANSparkMax rollerMotor;

    private ElevatorPositions elevatorPosition;

    private Map<ElevatorPositions, Double> positionValues;
    private Map<ElevatorPositions, Double> pivotPositionValues;

    public Elevator(){

        positionValues = new HashMap<>();
        positionValues.put(ElevatorPositions.PODIUM, ElevatorConstants.kElevatorPodiumPosition);
        positionValues.put(ElevatorPositions.AMP, ElevatorConstants.kElevatorAmpPosition);
        positionValues.put(ElevatorPositions.STOW, ElevatorConstants.kElevatorStowPosition);
        positionValues.put(ElevatorPositions.HANDOFF, ElevatorConstants.kElevatorHandoffPosition);

        pivotPositionValues = new HashMap<>();
        pivotPositionValues.put(ElevatorPositions.PODIUM, ElevatorConstants.kElevatorPodiumPivotPosition);
        pivotPositionValues.put(ElevatorPositions.AMP, ElevatorConstants.kElevatorAmpPivotPosition);
        pivotPositionValues.put(ElevatorPositions.STOW, ElevatorConstants.kElevatorStowPivotPosition);
        pivotPositionValues.put(ElevatorPositions.HANDOFF, ElevatorConstants.kElevatorHandoffPivotPosition);
        

        this.elevatorMotor1 = new CANSparkMax(ElevatorConstants.kElevatorMotor1Port, CANSparkMax.MotorType.kBrushless);
        this.elevatorMotor2 = new CANSparkMax(ElevatorConstants.kElevatorMotor2Port, CANSparkMax.MotorType.kBrushless);
        this.elevatorMotors = new MotorControlGroup(elevatorMotor1, elevatorMotor2);

        this.pivotMotor = new CANSparkMax(ElevatorConstants.kElevatorPivotMotorPort, CANSparkMax.MotorType.kBrushless);
        this.rollerMotor = new CANSparkMax(ElevatorConstants.kRollerMotorPort, CANSparkMax.MotorType.kBrushless);

        elevatorPIDController = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI, ElevatorConstants.kElevatorD);
        elevatorPivotPIDController = new PIDController(ElevatorConstants.kElevatorPivotP, ElevatorConstants.kElevatorPivotI, ElevatorConstants.kElevatorPivotD);
        
        this.setElevatorBrakeMode(true);
        this.setPivotBrakeMode(true);
        this.setRollerBrakeMode(true);

    }

    public void setElevatorPower(double power){
        elevatorMotors.setPower(power);
        elevatorPosition = ElevatorPositions.MANUAL;
    }

    public void setElevatorPosition(ElevatorPositions position){
        double positionValue = positionValues.get(position);
        elevatorMotors.setPosition(positionValue, elevatorPIDController);
        elevatorPosition = position;
    }

    public void setPivotPower(double power){
        pivotMotor.set(power);
    }

    public void setPivotPosition(ElevatorPositions position){
        double positionValue = pivotPositionValues.get(position);
        elevatorPivotPIDController.setSetpoint(positionValue);
        pivotMotor.set(elevatorPivotPIDController.calculate(pivotMotor.getEncoder().getPosition()));
    }

    public void setRollerPower(double power){
        rollerMotor.set(power);
    }

    public void setElevatorBrakeMode(boolean brakeMode){
        elevatorMotors.setDefaultBrakeMode(brakeMode);
    }

    public void setPivotBrakeMode(boolean brakeMode){
        pivotMotor.setIdleMode(brakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    public void setRollerBrakeMode(boolean brakeMode){
        rollerMotor.setIdleMode(brakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }



    public void setPosition(ElevatorPositions position){
        switch (position){
            case PODIUM:
                this.setElevatorPosition(position);
                this.setPivotPosition(position);
                elevatorPosition = position;
                break;
            case AMP:
                this.setElevatorPosition(position);
                this.setPivotPosition(position);
                elevatorPosition = position;
                break;
            case STOW:
                this.setElevatorPosition(position);
                this.setPivotPosition(position);
                elevatorPosition = position;
                break;
            case HANDOFF:
                this.setElevatorPosition(position);
                this.setPivotPosition(position);
                elevatorPosition = position;
                break;
            default:
                break;                
        }
    }

    public ElevatorPositions getElevatorPosition(){
        return elevatorPosition;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position", elevatorMotors.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevating Pivot Position", pivotMotor.getEncoder().getPosition());
    }

    
}
