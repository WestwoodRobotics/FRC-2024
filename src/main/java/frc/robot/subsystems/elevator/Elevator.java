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
import frc.robot.subsystems.vision.BeamBreak;

/**
 * The Elevator subsystem controls the elevator mechanism used for lifting the robot or game pieces.
 * It includes methods to control the elevator's motors, set its position, and read its sensors.
 */
public class Elevator extends SubsystemBase{

    private CANSparkMax primaryElevatorMotor;
    private CANSparkMax secondaryElevatorMotor;

    private PIDController elevatorPIDController;
    private PIDController elevatorPivotPIDController;

    private MotorControlGroup elevatorMotors;
    private CANSparkMax pivotMotor;
    private CANSparkMax rollerMotor;

    private ElevatorPositions elevatorPosition;
    private ElevatorPositions elevatorPivotPosition;

    private Map<ElevatorPositions, Double> ElevatorPositionValues;
    private Map<ElevatorPositions, Double> pivotPositionValues;

    private boolean isElevatorPIDControl = false;
    private boolean isPivotPIDCOntrol = false;

    public Elevator(){

        ElevatorPositionValues = new HashMap<>();
        ElevatorPositionValues.put(ElevatorPositions.PODIUM, ElevatorConstants.kElevatorPodiumPosition);
        ElevatorPositionValues.put(ElevatorPositions.AMP, ElevatorConstants.kElevatorAmpPosition);
        ElevatorPositionValues.put(ElevatorPositions.HOME, ElevatorConstants.kElevatorHomePosition);
        ElevatorPositionValues.put(ElevatorPositions.SOURCE, ElevatorConstants.kElevatorSourcePosition);
        ElevatorPositionValues.put(ElevatorPositions.AUTO_SHOOT, ElevatorConstants.kElevatorAutoShootPosition);
        ElevatorPositionValues.put(ElevatorPositions.HANDOFF, ElevatorConstants.kElevatorHandoffPosition);

        pivotPositionValues = new HashMap<>();
        pivotPositionValues.put(ElevatorPositions.PODIUM, ElevatorConstants.kElevatorPodiumPivotPosition);
        pivotPositionValues.put(ElevatorPositions.AMP, ElevatorConstants.kElevatorAmpPivotPosition);
        pivotPositionValues.put(ElevatorPositions.HOME, ElevatorConstants.kElevatorHomePivotPosition);
        pivotPositionValues.put(ElevatorPositions.SOURCE, ElevatorConstants.kElevatorSourcePivotPosition);
        pivotPositionValues.put(ElevatorPositions.AUTO_SHOOT, ElevatorConstants.kElevatorAutoShootPivotPosition);
        pivotPositionValues.put(ElevatorPositions.HANDOFF, ElevatorConstants.kElevatorHandoffPivotPosition);

        this.primaryElevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotor1Port, CANSparkMax.MotorType.kBrushless);
        this.secondaryElevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotor2Port, CANSparkMax.MotorType.kBrushless);
        this.primaryElevatorMotor.setInverted(true);
        this.secondaryElevatorMotor.setInverted(false );


        this.pivotMotor = new CANSparkMax(ElevatorConstants.kElevatorPivotMotorPort, CANSparkMax.MotorType.kBrushless);
        this.rollerMotor = new CANSparkMax(ElevatorConstants.kRollerMotorPort, CANSparkMax.MotorType.kBrushless);

        this.rollerMotor.setInverted(false); //TODO: Check
        this.pivotMotor.setInverted(false); //TODO: Check 
        this.pivotMotor.getEncoder().setPosition(0);


        elevatorPIDController = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI, ElevatorConstants.kElevatorD);
        elevatorPivotPIDController = new PIDController(ElevatorConstants.kElevatorPivotP, ElevatorConstants.kElevatorPivotI, ElevatorConstants.kElevatorPivotD);
        
        this.setElevatorBrakeMode(true);
        this.setPivotBrakeMode(true);
        this.setRollerBrakeMode(true);



    }

    public void setElevatorPower(double power){
        isElevatorPIDControl = false;

        primaryElevatorMotor.set(power);
        secondaryElevatorMotor.set(power);
        elevatorPosition = ElevatorPositions.MANUAL;
    }

    public void setElevatorPosition(ElevatorPositions position){
        double positionValue = ElevatorPositionValues.get(position);
        elevatorPIDController.setSetpoint(positionValue);
        System.out.println(positionValue);
        isElevatorPIDControl = true;
    }

    public double getTargetElevatorPositionEncoderValue(ElevatorPositions position)
    {
        return ElevatorPositionValues.get(position);
    }

    public void setPivotPower(double power){
        isPivotPIDCOntrol = false;
        pivotMotor.set(power);
        elevatorPivotPosition = ElevatorPositions.MANUAL;
    }

    public void setPivotPosition (ElevatorPositions positions){
        double positionValue = pivotPositionValues.get(positions);
        elevatorPivotPIDController.setSetpoint(positionValue);
        System.out.println(positionValue);
        isPivotPIDCOntrol = true;
    }

    public void resetEncoder(){
        pivotMotor.getEncoder().setPosition(0);
    }

    public boolean setElevatorPositionNOPID (ElevatorPositions positions){
        double positionValue = ElevatorPositionValues.get(positions);
        double currentPosition = primaryElevatorMotor.getEncoder().getPosition();
        if (positionValue < currentPosition){
            primaryElevatorMotor.set(-0.5);
            secondaryElevatorMotor.set(-0.5);
            System.out.println("Elevator Going");
            return false;
        } 
        else if (positionValue > currentPosition){
            primaryElevatorMotor.set(+0.5);
            secondaryElevatorMotor.set(+0.5);
            elevatorPosition = positions;
            System.out.println("Elevator Going");
            return false;
        }
        else if (positionValue == currentPosition){
            primaryElevatorMotor.set(0);
            secondaryElevatorMotor.set(0);
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

    public double getElevatorEncoderPosition(){
        return primaryElevatorMotor.getEncoder().getPosition();
        }

    public ElevatorPositions getPivotPosition(){
        return elevatorPivotPosition;
    }
    @Override
    public void periodic(){
        
        SmartDashboard.putNumber("Elevator Position", primaryElevatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevating Pivot Position", pivotMotor.getEncoder().getPosition());
        
        if (isElevatorPIDControl){
            primaryElevatorMotor.set(elevatorPIDController.calculate(primaryElevatorMotor.getEncoder().getPosition()));
            secondaryElevatorMotor.set(elevatorPIDController.calculate(secondaryElevatorMotor.getEncoder().getPosition()));
        }

        if (isPivotPIDCOntrol){
            double power = elevatorPivotPIDController.calculate(pivotMotor.getEncoder().getPosition());
            if(power > 0.5) {
                power = 0.5;
            }
            else if (power < -0.5) {
                power = -0.5;
            }
            pivotMotor.set(power);
            System.out.println("power: " + power);
            System.out.println("setpoint " + elevatorPivotPIDController.getSetpoint());
            System.out.println("encoder pose " + pivotMotor.getEncoder().getPosition());
        }
        
    }

    
}
