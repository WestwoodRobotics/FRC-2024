package frc.robot.subsystems.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.*;


import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.PIDController;

/**
 * A utility class for grouping multiple motor controllers for synchronized control.
 * This class allows for setting power, position, and other properties across multiple motors simultaneously.
 */
public class MotorControlGroup {

    private final CANSparkBase[] motors;

    /**
     * Constructor that takes in an array of CANSparkMax motors.
     * 
     * @param motors An array of CANSparkBase motors to be controlled together.
     */
    public MotorControlGroup(CANSparkBase... motors) {
        this.motors = motors;
    }

    /**
     * Returns the number of motors in the group.
     * 
     * @return The number of motors in the group.
     */
    public double getNumMotors() {
        return motors.length;
    }

    /**
     * Sets the power of all motors in the group to the given value.
     * 
     * @param power The power to set for all motors, between -1.0 and 1.0.
     */
    public void setPower(double power) {
        for (CANSparkBase motor : motors) {
            motor.set(power);
        }
    }

    /**
     * Sets the power of a specific motor in the group.
     * 
     * @param power The power to set for the specified motor, between -1.0 and 1.0.
     * @param motorIndex The index of the motor in the group to set the power for.
     */
    public void setPower(double power, int motorIndex) {
        motors[motorIndex].set(power);
    }

    /**
     * Sets the default brake mode for all motors in the control group.
     * 
     * @param brakeMode True to enable brake mode, false for coast mode.
     */
    public void setDefaultBrakeMode(boolean brakeMode){
        for (CANSparkBase motor : motors) {
            motor.setIdleMode(brakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
        }
    }

    /**
     * Sets the position of all motors in the group to the given value, with the given feedforward value.
     * 
     * @param position The target position for the motors.
     * @param kP The proportional gain for the PID controller.
     * @param kI The integral gain for the PID controller.
     * @param kD The derivative gain for the PID controller.
     * @param ff The feedforward value for the PID controller.
     */
    public void setPosition(double position, double kP, double kI, double kD, double ff) {
        for (CANSparkBase motor : motors) {
            motor.getPIDController().setP(kP);
            motor.getPIDController().setI(kI);
            motor.getPIDController().setD(kD);
            motor.getPIDController().setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
        }
    }

    /**
     * Sets the position of all motors in the group to the given value, with the given feedforward value.
     * 
     * @param position The target position for the motors.
     * @param ff The feedforward value for the PID controller.
     */
    public void setPosition(double position, double ff) {
        for (CANSparkBase motor : motors) {
            motor.getPIDController().setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
        }
    }

    /**
     * Sets the position of a specific motor in the group using a PID controller.
     * 
     * @param position The target position for the motor.
     * @param pidController The PID controller to use for setting the position.
     * @param motorNum The index of the motor in the group to set the position for.
     */
    public void setPosition(double position, PIDController pidController, int motorNum){ 
        CANSparkBase motor = motors[motorNum];
        this.setPower(pidController.calculate(motor.getEncoder().getPosition(), position), motorNum);
    }

    /**
     * Sets the position of all motors in the group using a PID controller.
     * 
     * @param position The target position for the motors.
     * @param pidController The PID controller to use for setting the position.
     */
    public void setPosition(double position, PIDController pidController){
        for (CANSparkBase motor : motors) {
            motor.set(pidController.calculate(motor.getEncoder().getPosition(), position));
        }
    }

    /**
     * Sets the inversion state of all motors in the group.
     * 
     * @param inverted True to invert the motor directions, false to use normal direction.
     */
    public void setInverted(boolean inverted){
        for (CANSparkBase motor : motors) {
            motor.setInverted(inverted);
        }
    }

    /**
     * Sets the inversion state of a specific motor in the group.
     * 
     * @param inverted True to invert the motor direction, false to use normal direction.
     * @param motorIndex The index of the motor in the group to set the inversion for.
     */
    public void setInverted(boolean inverted, int motorIndex){
        motors[motorIndex].setInverted(inverted);
    }
    
    /**
     * Returns the power of the first motor in the group.
     * 
     * @return The power of the first motor in the group.
     */
    public double getPower() {
        return motors[0].get();
    }

    /**
     * Returns the position of the first motor in the group.
     * 
     * @return The position of the first motor in the group.
     */
    public double getPosition() {
        return motors[0].getEncoder().getPosition();
    }

    /**
     * Sets the ramp rate for all motors in the group.
     * 
     * @param rate The ramp rate in seconds to go from 0 to full throttle.
     */
    public void setRampRate(double rate) {
        for (CANSparkBase motor : motors) {
            motor.setOpenLoopRampRate(rate);
            motor.setClosedLoopRampRate(rate);
        }
    }

    /**
     * Returns the average temperature of all motors in the group.
     * 
     * @return The average temperature of all motors in the group.
     */
    public double getAverageTemperature() {
        double totalTemp = 0.0;
        for (CANSparkBase motor : motors) {
            totalTemp += motor.getMotorTemperature();
        }
        return totalTemp / motors.length;
    }

    /**
     * Returns the average bus voltage of all motors in the group.
     * 
     * @return The average bus voltage of all motors in the group.
     */
    public double getAverageBusVoltage() {
        double totalVoltage = 0.0;
        for (CANSparkBase motor : motors) {
            totalVoltage += motor.getBusVoltage();
        }
        return totalVoltage / motors.length;
    }

    /**
     * Returns the average output current of all motors in the group.
     * 
     * @return The average output current of all motors in the group.
     */
    public double getAverageOutputCurrent() {
        double totalCurrent = 0.0;
        for (CANSparkBase motor : motors) {
            totalCurrent += motor.getOutputCurrent();
        }
        return totalCurrent / motors.length;
    }

    /**
     * Checks if any motor in the group has faults.
     * 
     * @return True if any motor has faults, false otherwise.
     */
    public boolean anyFaults() {
        for (CANSparkBase motor : motors) {
            if (motor.getFaults() > 0) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns the encoder of the first motor in the group.
     * 
     * @return The encoder of the first motor in the group.
     */
    public RelativeEncoder getEncoder() {
        return motors[0].getEncoder();
    }

}
