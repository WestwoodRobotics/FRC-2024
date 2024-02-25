package frc.robot.subsystems.elevator;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.utils.MotorControlGroup;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;

public class Elevator extends SubsystemBase{

    private CANSparkMax elevatorMotor1;
    private CANSparkMax elevatorMotor2;

    private MotorControlGroup elevatorMotors;
    private CANSparkMax pivotMotor;
    private CANSparkMax rollerMotor;

    private ElevatorPositions elevatorPosition;

    private Map<ElevatorPositions, Double> positionValues;

    public Elevator(){

        positionValues = new HashMap<>();
        positionValues.put(ElevatorPositions.PODIUM, ElevatorConstants.kElevatorPodiumPosition);
        positionValues.put(ElevatorPositions.AMP, ElevatorConstants.kElevatorAmpPosition);
        positionValues.put(ElevatorPositions.STOW, ElevatorConstants.kElevatorStowPosition);

        this.elevatorMotor1 = new CANSparkMax(ElevatorConstants.kElevatorMotor1Port, CANSparkMax.MotorType.kBrushless);
        this.elevatorMotor2 = new CANSparkMax(ElevatorConstants.kElevatorMotor2Port, CANSparkMax.MotorType.kBrushless);
        this.elevatorMotors = new MotorControlGroup(elevatorMotor1, elevatorMotor2);

        this.pivotMotor = new CANSparkMax(ElevatorConstants.kPivotMotorPort, CANSparkMax.MotorType.kBrushless);
        this.rollerMotor = new CANSparkMax(ElevatorConstants.kRollerMotorPort, CANSparkMax.MotorType.kBrushless);
    }

    public void setElevatorPower(double power){
        elevatorMotors.setPower(power);
        elevatorPosition = ElevatorPositions.MANUAL;
    }

    public void setPivotPower(double power){
        pivotMotor.set(power);
    }

    public void setPivotPosition(double position, double kP, double kI, double kD, double ff){
        pivotMotor.getPIDController().setP(kP);
        pivotMotor.getPIDController().setI(kI);
        pivotMotor.getPIDController().setD(kD);
        pivotMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, ff);

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
                elevatorMotors.setPosition(positionValues.get(position), ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD, ElevatorConstants.kFF);
                this.setPivotPosition(0, 0, 0, 0, 0);
                elevatorPosition = position;
                break;
            case AMP:
                elevatorMotors.setPosition(positionValues.get(position), ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD, ElevatorConstants.kFF);
                this.setPivotPosition(0, 0, 0, 0, 0);
                elevatorPosition = position;
                break;
            case STOW:
                elevatorMotors.setPosition(positionValues.get(position), ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD, ElevatorConstants.kFF);
                this.setPivotPosition(0, 0, 0, 0, 0);
                elevatorPosition = position;
                break;
            default:
                break;
        }
    }

    public ElevatorPositions getElevatorPosition(){
        return elevatorPosition;
    }





    
}
