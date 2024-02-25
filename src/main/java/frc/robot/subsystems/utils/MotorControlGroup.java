
package frc.robot.subsystems.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.*;


import com.revrobotics.SparkPIDController.ArbFFUnits;

public class MotorControlGroup {

    private final CANSparkBase[] motors;


    // Constructor that takes in an array of CANSparkMax motors
    public MotorControlGroup(CANSparkBase... motors) {
        this.motors = motors;
    }

    // Returns the number of motors in the group
    public double getNumMotors() {
        return motors.length;
    }

    // Sets the power of all motors in the group to the given value
    public void setPower(double power) {
        for (CANSparkBase motor : motors) {
            motor.set(power);
        }
    }


    //Set the defaultBrakeMode of all motors in the control group
    public void setDefaultBrakeMode(boolean brakeMode){
        for (CANSparkBase motor : motors) {
            motor.setIdleMode(brakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
        }
    }

    // Sets the position of all motors in the group to the given value, with the given feedforward value
    public void setPosition(double position, double kP, double kI, double kD, double ff) {
        for (CANSparkBase motor : motors) {
            motor.getPIDController().setP(kP);
            motor.getPIDController().setI(kI);
            motor.getPIDController().setD(kD);
            motor.getPIDController().setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
        }
    }

    // Sets the position of all motors in the group to the given value, with the given feedforward value
    public void setPosition(double position, double ff) {
        for (CANSparkBase motor : motors) {
            motor.getPIDController().setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
        }
    }

    public void setInverted(boolean inverted){
        for (CANSparkBase motor : motors) {
            motor.setInverted(inverted);
        }
    }

    public void setInverted(boolean inverted, int motorIndex){
        motors[motorIndex].setInverted(inverted);
    }
    
    // Returns the power of the first motor in the group (all motors should be the same)
    public double getPower() {
        return motors[0].get();
    }

    // Returns the position of the first motor in the group (all motors should be the same)
    public double getPosition() {
        return motors[0].getEncoder().getPosition();
    }

    public void setRampRate(double rate) {
        for (CANSparkBase motor : motors) {
            motor.setOpenLoopRampRate(rate);
            motor.setClosedLoopRampRate(rate);
        }
    }

    public double getAverageTemperature() {
        double totalTemp = 0.0;
        for (CANSparkBase motor : motors) {
            totalTemp += motor.getMotorTemperature();
        }
        return totalTemp / motors.length;
    }

    public double getAverageBusVoltage() {
        double totalVoltage = 0.0;
        for (CANSparkBase motor : motors) {
            totalVoltage += motor.getBusVoltage();
        }
        return totalVoltage / motors.length;
    }

    public double getAverageOutputCurrent() {
        double totalCurrent = 0.0;
        for (CANSparkBase motor : motors) {
            totalCurrent += motor.getOutputCurrent();
        }
        return totalCurrent / motors.length;
    }

    public boolean anyFaults() {
        for (CANSparkBase motor : motors) {
            if (motor.getFaults() > 0) {
                return true;
            }
        }
        return false;
    }

}
    
