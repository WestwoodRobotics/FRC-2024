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
        this.elevatorMotor1.setInverted(true);

        this.pivotMotor = new CANSparkMax(ElevatorConstants.kElevatorPivotMotorPort, CANSparkMax.MotorType.kBrushless);
        this.rollerMotor = new CANSparkMax(ElevatorConstants.kRollerMotorPort, CANSparkMax.MotorType.kBrushless);

        elevatorPIDController = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI, ElevatorConstants.kElevatorD);
        elevatorPivotPIDController = new PIDController(ElevatorConstants.kElevatorPivotP, ElevatorConstants.kElevatorPivotI, ElevatorConstants.kElevatorPivotD);
        
        this.setElevatorBrakeMode(true);
        this.setPivotBrakeMode(true);
        this.setRollerBrakeMode(true);
    }

    public void setElevatorPower(double power){
        elevatorMotor1.set(power);
        elevatorMotor2.set(power);
        elevatorPosition = ElevatorPositions.MANUAL;
    }

    public void setElevatorPosition(ElevatorPositions position){
        double positionValue = positionValues.get(position);
        elevatorPIDController.setSetpoint(positionValue);
        elevatorMotor1.set(elevatorPIDController.calculate(elevatorMotor1.getEncoder().getPosition()));
        elevatorMotor2.set(elevatorPIDController.calculate(elevatorMotor1.getEncoder().getPosition()));
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

    public boolean setPivotPositionNOPID (ElevatorPositions positions){
        double positionValue = pivotPositionValues.get(positions);
        double currentPosition = pivotMotor.getEncoder().getPosition();
        if (positionValue < currentPosition){
         pivotMotor.set(0.5);
         System.out.println("Pivot Going");
         return false;
        } 
        else if (positionValue > currentPosition){
            pivotMotor.set(-0.5);
            elevatorPosition = positions;
            System.out.println("Pivot Going");
            return false;
        }
        else if (positionValue == currentPosition){
            pivotMotor.set(0);
            elevatorPosition = positions;
            System.out.println("Pivot Reached");
            return true;
        }


        return false;
    }

    public void setRollerPower(double power){
        rollerMotor.set(power);
    }

    public void setElevatorBrakeMode(boolean brakeMode){
        //elevatorMotor.setDefaultBrakeMode(brakeMode);
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
        SmartDashboard.putNumber("Elevator Position", elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevating Pivot Position", pivotMotor.getEncoder().getPosition());

        boolean motor1Follower = elevatorMotor1.isFollower();
        boolean motor2Follower = elevatorMotor2.isFollower();

        SmartDashboard.putBoolean("Elevator Motor 1 Is Follower", motor1Follower);
        SmartDashboard.putBoolean("Elevator Motor 2 Is Follower", motor2Follower);
        
        
    }

    
}
