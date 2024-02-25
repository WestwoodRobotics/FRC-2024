package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.utils.MotorControlGroup;

public class Elevator extends SubsystemBase{

    private CANSparkMax elevatorMotor1;
    private CANSparkMax elevatorMotor2;

    private MotorControlGroup elevatorMotors;
    private CANSparkMax pivotMotor;
    private CANSparkMax rollerMotor;


    public Elevator(){
        this.elevatorMotor1 = new CANSparkMax(ElevatorConstants.kElevatorMotor1Port, CANSparkMax.MotorType.kBrushless);
        this.elevatorMotor2 = new CANSparkMax(ElevatorConstants.kElevatorMotor2Port, CANSparkMax.MotorType.kBrushless);
        this.elevatorMotors = new MotorControlGroup(elevatorMotor1, elevatorMotor2);

        this.pivotMotor = new CANSparkMax(ElevatorConstants.kPivotMotorPort, CANSparkMax.MotorType.kBrushless);
        this.rollerMotor = new CANSparkMax(ElevatorConstants.kRollerMotorPort, CANSparkMax.MotorType.kBrushless);
    }

    public void setElevatorPower(double power){
        elevatorMotors.setPower(power);
    }

    public void setPivotPower(double power){
        pivotMotor.set(power);
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



    
}
