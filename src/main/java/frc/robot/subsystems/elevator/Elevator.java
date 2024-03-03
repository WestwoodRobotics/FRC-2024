package frc.robot.subsystems.elevator;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

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
    private ElevatorPositions elevatorPivotPosition;

    private Map<ElevatorPositions, Double> ElevatorPositionValues;
    private Map<ElevatorPositions, Double> pivotPositionValues;

    public Elevator(){

        ElevatorPositionValues = new HashMap<>();
        ElevatorPositionValues.put(ElevatorPositions.PODIUM, ElevatorConstants.kElevatorPodiumPosition);
        ElevatorPositionValues.put(ElevatorPositions.AMP, ElevatorConstants.kElevatorAmpPosition);
        ElevatorPositionValues.put(ElevatorPositions.HOME, ElevatorConstants.kElevatorHomePosition);
        ElevatorPositionValues.put(ElevatorPositions.HANDOFF, ElevatorConstants.kElevatorHandoffPosition);
        ElevatorPositionValues.put(ElevatorPositions.SOURCE, ElevatorConstants.kElevatorSourcePosition);

        pivotPositionValues = new HashMap<>();
        pivotPositionValues.put(ElevatorPositions.PODIUM, ElevatorConstants.kElevatorPodiumPivotPosition);
        pivotPositionValues.put(ElevatorPositions.AMP, ElevatorConstants.kElevatorAmpPivotPosition);
        pivotPositionValues.put(ElevatorPositions.HOME, ElevatorConstants.kElevatorHomePivotPosition);
        pivotPositionValues.put(ElevatorPositions.HANDOFF, ElevatorConstants.kElevatorHandoffPivotPosition);
        pivotPositionValues.put(ElevatorPositions.SOURCE, ElevatorConstants.kElevatorSourcePivotPosition);
        

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

    public boolean setElevatorPosition(ElevatorPositions position){
        double positionValue = ElevatorPositionValues.get(position);
        double currentPosition = elevatorMotor1.getEncoder().getPosition();
        
        if (positionValue < currentPosition){
            elevatorPIDController.setSetpoint(positionValue);
            elevatorMotor1.set(elevatorPIDController.calculate(elevatorMotor1.getEncoder().getPosition()));
            elevatorMotor2.set(elevatorPIDController.calculate(elevatorMotor2.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));
            System.out.println("Pivot Going 1");
            return false;
        } 
        if(positionValue > currentPosition){
            elevatorPIDController.setSetpoint(positionValue);
            elevatorMotor1.set(elevatorPIDController.calculate(elevatorMotor1.getEncoder().getPosition()));
            elevatorMotor2.set(elevatorPIDController.calculate(elevatorMotor2.getEncoder().getPosition()));
            System.out.println("Pivot Going 1");
            return false;
        }
        if (positionValue == currentPosition){
            elevatorMotor1.set(0);
            elevatorMotor2.set(0);
            //elevatorPivotPosition = positions;
            System.out.println("Pivot Reached");
            return true;
        }
        return false;
    }

    public void setPivotPower(double power){
        pivotMotor.set(power);
        elevatorPivotPosition = ElevatorPositions.MANUAL;
    }

    public boolean setPivotPosition (ElevatorPositions positions){
        double positionValue = pivotPositionValues.get(positions);
        double currentPosition = pivotMotor.getEncoder().getPosition();
        
        if (positionValue < currentPosition){
            elevatorPivotPIDController.setSetpoint(positionValue);
            pivotMotor.set(elevatorPivotPIDController.calculate(currentPosition));
            System.out.println("Pivot Going 1");
            return false;
        } 

        if(positionValue > currentPosition){
            elevatorPivotPIDController.setSetpoint(positionValue);
            pivotMotor.set(elevatorPivotPIDController.calculate(currentPosition));
            System.out.println("Pivot Going 2");

        }

        if ((positionValue == currentPosition)){
            pivotMotor.set(0);
            //elevatorPivotPosition = positions;
            System.out.println("Pivot Reached");
            return true;
        }
        return false;
    }

    public boolean setElevatorPositionNOPID (ElevatorPositions positions){
        double positionValue = ElevatorPositionValues.get(positions);
        double currentPosition = elevatorMotor1.getEncoder().getPosition();
        if (positionValue < currentPosition){
            elevatorMotor1.set(-0.5);
            elevatorMotor2.set(-0.5);
            System.out.println("Elevator Going");
            return false;
        } 
        else if (positionValue > currentPosition){
            elevatorMotor1.set(+0.5);
            elevatorMotor2.set(+0.5);
            elevatorPosition = positions;
            System.out.println("Elevator Going");
            return false;
        }
        else if (positionValue == currentPosition){
            elevatorMotor1.set(0);
            elevatorMotor2.set(0);
            elevatorPosition = positions;
            System.out.println("Elevator Reached");
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
        this.setPivotPosition(position);
        elevatorPosition = position;
    }

    public ElevatorPositions getElevatorPosition(){
        return elevatorPosition;
    }

    public ElevatorPositions getPivotPosition(){
        return elevatorPivotPosition;
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position", elevatorMotor1.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
        SmartDashboard.putNumber("Elevating Pivot Position", pivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());

        boolean motor1Follower = elevatorMotor1.isFollower();
        boolean motor2Follower = elevatorMotor2.isFollower();

        SmartDashboard.putBoolean("Elevator Motor 1 Is Follower", motor1Follower);
        SmartDashboard.putBoolean("Elevator Motor 2 Is Follower", motor2Follower);
        
        
    }

    
}
