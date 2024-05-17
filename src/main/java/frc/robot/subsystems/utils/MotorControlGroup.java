package frc.robot.subsystems.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.*;


import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.PIDController;

/**
 * Manages a group of motors, allowing synchronized control over their power, position, and settings.
 * This class is useful for managing multiple motors that drive a single mechanism in unison.
 */
public class MotorControlGroup {

    private final CANSparkBase[] motorGroup; // Renamed from motors to motorGroup for clarity

    /**
     * Constructor that takes in an array of CANSparkMax motors
     * @param motors An array of CANSparkBase motors that will be controlled as a group.
     */
    public MotorControlGroup(CANSparkBase... motors) {
        this.motorGroup = motors;
    }

    /**
     * Returns the number of motors in the group
     * @return The number of motors in the motor group.
     */
    public double getNumMotors() {
        return motorGroup.length;
    }

    /**
     * Sets the power of all motors in the group to the given value
     * @param power The power level to set for all motors in the group.
     */
    public void setPower(double power) {
        for (CANSparkBase motor : motorGroup) {
            motor.set(power);
        }
    }

    /**
     * Sets the power of a specific motor in the group to the given value
     * @param power The power level to set for the specified motor.
     * @param motorIndex The index of the motor in the motor group whose power is to be set.
     */
    public void setPower(double power, int motorIndex) {
        motorGroup[motorIndex].set(power);
    }

    /**
     * Set the default brake mode of all motors in the control group
     * @param brakeMode True to enable brake mode, false for coast mode.
     */
    public void setDefaultBrakeMode(boolean brakeMode){
        for (CANSparkBase motor : motorGroup) {
            motor.setIdleMode(brakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
        }
    }

    /**
     * Sets the position of all motors in the group to the given value, with the given feedforward value
     * @param position The target position for the motors.
     * @param kP The proportional gain for the PID controller.
     * @param kI The integral gain for the PID controller.
     * @param kD The derivative gain for the PID controller.
     * @param ff The feedforward value for the PID controller.
     */
    public void setPosition(double position, double kP, double kI, double kD, double ff) {
        for (CANSparkBase motor : motorGroup) {
            motor.getPIDController().setP(kP);
            motor.getPIDController().setI(kI);
            motor.getPIDController().setD(kD);
            motor.getPIDController().setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
        }
    }

    /**
     * Sets the position of all motors in the group to the given value, with the given feedforward value
     * @param position The target position for the motors.
     * @param ff The feedforward value for the PID controller.
     */
    public void setPosition(double position, double ff) {
        for (CANSparkBase motor : motorGroup) {
            motor.getPIDController().setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
        }
    }

    /**
     * Sets the position of a specific motor in the group using a PID controller
     * @param position The target position for the motor.
     * @param pidController The PID controller to use for setting the position.
     * @param motorNum The index of the motor in the motor group whose position is to be set.
     */
    public void setPosition(double position, PIDController pidController, int motorNum){ 
        CANSparkBase motor = motorGroup[motorNum];
        this.setPower(pidController.calculate(motor.getEncoder().getPosition(), position), motorNum);
    }

    /**
     * Sets the position of all motors in the group using a PID controller
     * @param position The target position for the motors.
     * @param pidController The PID controller to use for setting the position.
     */
    public void setPosition(double position, PIDController pidController){
        for (CANSparkBase motor : motorGroup) {
            motor.set(pidController.calculate(motor.getEncoder().getPosition(), position));
        }
    }

    /**
     * Sets the inversion state of all motors in the group.
     * @param inverted True to invert the motor directions, false to use normal direction.
     */
    public void setInverted(boolean inverted){
        for (CANSparkBase motor : motorGroup) {
            motor.setInverted(inverted);
        }
    }

    /**
     * Sets the inversion state of a specific motor in the group.
     * @param inverted True to invert the motor direction, false to use normal direction.
     * @param motorIndex The index of the motor in the motor group whose inversion state is to be set.
     */
    public void setInverted(boolean inverted, int motorIndex){
        motorGroup[motorIndex].setInverted(inverted);
    }
    
    /**
     * Returns the power of the first motor in the group (all motors should be the same)
     * @return The current power level of the first motor in the group.
     */
    public double getPower() {
        return motorGroup[0].get();
    }

    /**
     * Returns the position of the first motor in the group (all motors should be the same)
     * @return The current position of the first motor in the group.
     */
    public double getPosition() {
        return motorGroup[0].getEncoder().getPosition();
    }

    /**
     * Sets the ramp rate for all motors in the group.
     * @param rate The ramp rate in seconds to go from 0 to full throttle.
     */
    public void setRampRate(double rate) {
        for (CANSparkBase motor : motorGroup) {
            motor.setOpenLoopRampRate(rate);
            motor.setClosedLoopRampRate(rate);
        }
    }

    /**
     * Returns the average temperature of all motors in the group.
     * @return The average temperature of the motors in the group.
     */
    public double getAverageTemperature() {
        double totalTemp = 0.0;
        for (CANSparkBase motor : motorGroup) {
            totalTemp += motor.getMotorTemperature();
        }
        return totalTemp / motorGroup.length;
    }

    /**
     * Returns the average bus voltage of all motors in the group.
     * @return The average bus voltage of the motors in the group.
     */
    public double getAverageBusVoltage() {
        double totalVoltage = 0.0;
        for (CANSparkBase motor : motorGroup) {
            totalVoltage += motor.getBusVoltage();
        }
        return totalVoltage / motorGroup.length;
    }

    /**
     * Returns the average output current of all motors in the group.
     * @return The average output current of the motors in the group.
     */
    public double getAverageOutputCurrent() {
        double totalCurrent = 0.0;
        for (CANSparkBase motor : motorGroup) {
            totalCurrent += motor.getOutputCurrent();
        }
        return totalCurrent / motorGroup.length;
    }

    /**
     * Checks if any motor in the group has faults.
     * @return True if any motor in the group has faults, false otherwise.
     */
    public boolean anyFaults() {
        for (CANSparkBase motor : motorGroup) {
            if (motor.getFaults() > 0) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns the encoder of the first motor in the group.
     * @return The encoder of the first motor in the group.
     */
    public RelativeEncoder getEncoder() {
        return motorGroup[0].getEncoder();
    }

}
