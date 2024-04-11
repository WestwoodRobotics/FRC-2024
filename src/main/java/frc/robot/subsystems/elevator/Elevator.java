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

        this.elevatorMotor1 = new CANSparkMax(ElevatorConstants.kElevatorMotor1Port, CANSparkMax.MotorType.kBrushless);
        this.elevatorMotor2 = new CANSparkMax(ElevatorConstants.kElevatorMotor2Port, CANSparkMax.MotorType.kBrushless);
        this.elevatorMotor1.setInverted(true);
        this.elevatorMotor2.setInverted(false );


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

        elevatorMotor1.set(power);
        elevatorMotor2.set(power);
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
        // if (Math.abs(positionValue - currentPosition) <= 0.05){
        //     pivotMotor.set(0);
        //     //elevatorPivotPosition = positions;
        //     System.out.println("Pivot Reached");
        //     return true;
        // }
        // else if (positionValue < currentPosition){
            
        // } 
        // else if(positionValue > currentPosition){
        //     elevatorPivotPIDController.setSetpoint(positionValue);
        //     double power = elevatorPivotPIDController.calculate(currentPosition);
        //     if(power > 0.5) {
        //         power = 0.5;
        //     }
        //     else if (power < -0.5) {
        //         power = -0.5;
        //     }
        //     pivotMotor.set(-1*power);
        //     return false;
        // }
        // return false;
    }

    public void resetEncoder(){
        pivotMotor.getEncoder().setPosition(0);
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

    public double getElevatorEncoderPosition(){
        return elevatorMotor1.getEncoder().getPosition();
        }

    public ElevatorPositions getPivotPosition(){
        return elevatorPivotPosition;
    }
    @Override
    public void periodic(){
        
        SmartDashboard.putNumber("Elevator Position", elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevating Pivot Position", pivotMotor.getEncoder().getPosition());
        
        if (isElevatorPIDControl){
            elevatorMotor1.set(elevatorPIDController.calculate(elevatorMotor1.getEncoder().getPosition()));
            elevatorMotor2.set(elevatorPIDController.calculate(elevatorMotor2.getEncoder().getPosition()));
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
