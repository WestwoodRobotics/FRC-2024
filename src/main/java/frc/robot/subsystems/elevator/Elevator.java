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

    private CANSparkMax primaryElevatorMotorController;
    private CANSparkMax secondaryElevatorMotorController;

    private PIDController elevatorPositionPIDController;
    private PIDController elevatorPivotPositionPIDController;

    private MotorControlGroup elevatorMotorGroup;
    private CANSparkMax elevatorPivotMotorController;
    private CANSparkMax elevatorRollerMotorController;

    private ElevatorPositions elevatorPosition;
    private ElevatorPositions elevatorPivotPosition;

    private Map<ElevatorPositions, Double> elevatorPositionTargetValues;
    private Map<ElevatorPositions, Double> elevatorPivotPositionTargetValues;

    private boolean isElevatorPositionPIDControlEnabled = false;
    private boolean isPivotPIDControl = false;

    public Elevator(){

        elevatorPositionTargetValues = new HashMap<>();
        elevatorPositionTargetValues.put(ElevatorPositions.PODIUM, ElevatorConstants.kElevatorPodiumPosition);
        elevatorPositionTargetValues.put(ElevatorPositions.AMP, ElevatorConstants.kElevatorAmpPosition);
        elevatorPositionTargetValues.put(ElevatorPositions.HOME, ElevatorConstants.kElevatorHomePosition);
        elevatorPositionTargetValues.put(ElevatorPositions.SOURCE, ElevatorConstants.kElevatorSourcePosition);
        elevatorPositionTargetValues.put(ElevatorPositions.AUTO_SHOOT, ElevatorConstants.kElevatorAutoShootPosition);
        elevatorPositionTargetValues.put(ElevatorPositions.HANDOFF, ElevatorConstants.kElevatorHandoffPosition);

        elevatorPivotPositionTargetValues = new HashMap<>();
        elevatorPivotPositionTargetValues.put(ElevatorPositions.PODIUM, ElevatorConstants.kElevatorPodiumPivotPosition);
        elevatorPivotPositionTargetValues.put(ElevatorPositions.AMP, ElevatorConstants.kElevatorAmpPivotPosition);
        elevatorPivotPositionTargetValues.put(ElevatorPositions.HOME, ElevatorConstants.kElevatorHomePivotPosition);
        elevatorPivotPositionTargetValues.put(ElevatorPositions.SOURCE, ElevatorConstants.kElevatorSourcePivotPosition);
        elevatorPivotPositionTargetValues.put(ElevatorPositions.AUTO_SHOOT, ElevatorConstants.kElevatorAutoShootPivotPosition);
        elevatorPivotPositionTargetValues.put(ElevatorPositions.HANDOFF, ElevatorConstants.kElevatorHandoffPivotPosition);

        this.primaryElevatorMotorController = new CANSparkMax(ElevatorConstants.kElevatorMotor1Port, CANSparkMax.MotorType.kBrushless);
        this.secondaryElevatorMotorController = new CANSparkMax(ElevatorConstants.kElevatorMotor2Port, CANSparkMax.MotorType.kBrushless);
        this.primaryElevatorMotorController.setInverted(true);
        this.secondaryElevatorMotorController.setInverted(false );


        this.elevatorPivotMotorController = new CANSparkMax(ElevatorConstants.kElevatorPivotMotorPort, CANSparkMax.MotorType.kBrushless);
        this.elevatorRollerMotorController = new CANSparkMax(ElevatorConstants.kRollerMotorPort, CANSparkMax.MotorType.kBrushless);

        this.elevatorRollerMotorController.setInverted(false); //TODO: Check
        this.elevatorPivotMotorController.setInverted(false); //TODO: Check 
        this.elevatorPivotMotorController.getEncoder().setPosition(0);


        elevatorPositionPIDController = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI, ElevatorConstants.kElevatorD);
        elevatorPivotPositionPIDController = new PIDController(ElevatorConstants.kElevatorPivotP, ElevatorConstants.kElevatorPivotI, ElevatorConstants.kElevatorPivotD);
        
        this.setElevatorBrakeMode(true);
        this.setPivotBrakeMode(true);
        this.setRollerBrakeMode(true);



    }

    /**
     * Sets the power for the elevator motors.
     * @param power The power level to set for the elevator motors.
     */
    public void setElevatorPower(double power){
        isElevatorPositionPIDControlEnabled = false;

        primaryElevatorMotorController.set(power);
        secondaryElevatorMotorController.set(power);
        elevatorPosition = ElevatorPositions.MANUAL;
    }

    /**
     * Sets the elevator to a specified position.
     * @param position The target position for the elevator.
     */
    public void setElevatorPosition(ElevatorPositions position){
        double positionValue = elevatorPositionTargetValues.get(position);
        elevatorPositionPIDController.setSetpoint(positionValue);
        System.out.println(positionValue);
        isElevatorPositionPIDControlEnabled = true;
    }

    /**
     * Gets the encoder value for a target elevator position.
     * @param position The elevator position to get the encoder value for.
     * @return The encoder value for the specified elevator position.
     */
    public double getTargetElevatorPositionEncoderValue(ElevatorPositions position)
    {
        return elevatorPositionTargetValues.get(position);
    }

    /**
     * Sets the power for the pivot motor.
     * @param power The power level to set for the pivot motor.
     */
    public void setPivotPower(double power){
        isPivotPIDControl = false;
        elevatorPivotMotorController.set(power);
        elevatorPivotPosition = ElevatorPositions.MANUAL;
    }

    /**
     * Sets the pivot to a specified position.
     * @param positions The target position for the pivot.
     */
    public void setPivotPosition (ElevatorPositions positions){
        double positionValue = elevatorPivotPositionTargetValues.get(positions);
        elevatorPivotPositionPIDController.setSetpoint(positionValue);
        System.out.println(positionValue);
        isPivotPIDControl = true;
    }

    /**
     * Resets the encoder for the pivot motor.
     */
    public void resetEncoder(){
        elevatorPivotMotorController.getEncoder().setPosition(0);
    }

    /**
     * Sets the elevator to a specified position without using PID control.
     * @param positions The target position for the elevator.
     * @return True if the elevator has reached the target position, false otherwise.
     */
    public boolean setElevatorPositionNOPID (ElevatorPositions positions){
        double positionValue = elevatorPositionTargetValues.get(positions);
        double currentPosition = primaryElevatorMotorController.getEncoder().getPosition();
        if (positionValue < currentPosition){
            primaryElevatorMotorController.set(-0.5);
            secondaryElevatorMotorController.set(-0.5);
            System.out.println("Elevator Going");
            return false;
        } 
        else if (positionValue > currentPosition){
            primaryElevatorMotorController.set(+0.5);
            secondaryElevatorMotorController.set(+0.5);
            elevatorPosition = positions;
            System.out.println("Elevator Going");
            return false;
        }
        else if (positionValue == currentPosition){
            primaryElevatorMotorController.set(0);
            secondaryElevatorMotorController.set(0);
            elevatorPosition = positions;
            System.out.println("Elevator Reached");
            return true;
        }
        return false;
    }


    /**
     * Sets the power for the roller motor.
     * @param power The power level to set for the roller motor.
     */
    public void setRollerPower(double power){
            elevatorRollerMotorController.set(power);
    }

    /**
     * Sets the brake mode for the elevator motors.
     * @param brakeMode True to enable brake mode, false for coast mode.
     */
    public void setElevatorBrakeMode(boolean brakeMode){
        //elevatorMotor.setDefaultBrakeMode(brakeMode);
    }

    /**
     * Sets the brake mode for the pivot motor.
     * @param brakeMode True to enable brake mode, false for coast mode.
     */
    public void setPivotBrakeMode(boolean brakeMode){
        elevatorPivotMotorController.setIdleMode(brakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    /**
     * Sets the brake mode for the roller motor.
     * @param brakeMode True to enable brake mode, false for coast mode.
     */
    public void setRollerBrakeMode(boolean brakeMode){
        elevatorRollerMotorController.setIdleMode(brakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    /**
     * Sets the position of the elevator.
     * @param position The target position for the elevator.
     */
    public void setPosition(ElevatorPositions position){
        this.setPivotPosition(position);
        elevatorPosition = position;
    }

    /**
     * Gets the current position of the elevator.
     * @return The current position of the elevator.
     */
    public ElevatorPositions getElevatorPosition(){
        return elevatorPosition;
    }

    /**
     * Gets the current encoder position of the elevator.
     * @return The current encoder position of the elevator.
     */
    public double getElevatorEncoderPosition(){
        return primaryElevatorMotorController.getEncoder().getPosition();
        }

    /**
     * Gets the current position of the pivot.
     * @return The current position of the pivot.
     */
    public ElevatorPositions getPivotPosition(){
        return elevatorPivotPosition;
    }
    @Override
    public void periodic(){
        
        SmartDashboard.putNumber("Elevator Position", primaryElevatorMotorController.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevating Pivot Position", elevatorPivotMotorController.getEncoder().getPosition());
        
        if (isElevatorPositionPIDControlEnabled){
            primaryElevatorMotorController.set(elevatorPositionPIDController.calculate(primaryElevatorMotorController.getEncoder().getPosition()));
            secondaryElevatorMotorController.set(elevatorPositionPIDController.calculate(secondaryElevatorMotorController.getEncoder().getPosition()));
        }

        if (isPivotPIDControl){
            double power = elevatorPivotPositionPIDController.calculate(elevatorPivotMotorController.getEncoder().getPosition());
            if(power > 0.5) {
                power = 0.5;
            }
            else if (power < -0.5) {
                power = -0.5;
            }
            elevatorPivotMotorController.set(power);
            System.out.println("power: " + power);
            System.out.println("setpoint " + elevatorPivotPositionPIDController.getSetpoint());
            System.out.println("encoder pose " + elevatorPivotMotorController.getEncoder().getPosition());
        }
        
    }

    
}
