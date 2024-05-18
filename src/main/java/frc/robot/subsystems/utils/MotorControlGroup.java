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

    private final CANSparkBase[] controlledMotors; // Renamed from motors to controlledMotors for clarity

    /**
     * Constructor that takes in an array of CANSparkMax motors
     * @param motors An array of CANSparkBase motors that will be controlled as a group.
     */
    public MotorControlGroup(CANSparkBase... motors) {
        this.controlledMotors = motors;
    }

    /**
     * Returns the number of motors in the group
     * @return The number of motors in the motor group.
     */
    public double getNumMotors() {
        return controlledMotors.length;
    }

    /**
     * Sets the power of all motors in the group to the given value
     * @param power The power level to set for all motors in the group.
     */
    public void setMotorPower(double power) {
        for (CANSparkBase motor : controlledMotors) {
            motor.set(power);
        }
    }

    /**
     * Sets the power of a specific motor in the group to the given value
     * @param power The power level to set for the specified motor.
     * @param motorIndex The index of the motor in the motor group whose power is to be set.
     */
    public void setMotorPower(double power, int motorIndex) {
        controlledMotors[motorIndex].set(power);
    }

    /**
     * Set the default brake mode of all motors in the control group
     * @param brakeMode True to enable brake mode, false for coast mode.
     */
    public void setMotorsDefaultBrakeMode(boolean brakeMode){
        for (CANSparkBase motor : controlledMotors) {
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
    public void setMotorPosition(double position, double kP, double kI, double kD, double ff) {
        for (CANSparkBase motor : controlledMotors) {
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
    public void setMotorPosition(double position, double ff) {
        for (CANSparkBase motor : controlledMotors) {
            motor.getPIDController().setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
        }
    }

    /**
     * Sets the position of a specific motor in the group using a PID controller
     * @param position The target position for the motor.
     * @param pidController The PID controller to use for setting the position.
     * @param motorNum The index of the motor in the motor group whose position is to be set.
     */
    public void setMotorPosition(double position, PIDController pidController, int motorNum){ 
        CANSparkBase motor = controlledMotors[motorNum];
        this.setMotorPower(pidController.calculate(motor.getEncoder().getPosition(), position), motorNum);
    }

    /**
     * Sets the position of all motors in the group using a PID controller
     * @param position The target position for the motors.
     * @param pidController The PID controller to use for setting the position.
     */
    public void setMotorPosition(double position, PIDController pidController){
        for (CANSparkBase motor : controlledMotors) {
            motor.set(pidController.calculate(motor.getEncoder().getPosition(), position));
        }
    }

    /**
     * Sets the inversion state of all motors in the group.
     * @param inverted True to invert the motor directions, false to use normal direction.
     */
    public void setMotorsInverted(boolean inverted){
        for (CANSparkBase motor : controlledMotors) {
            motor.setInverted(inverted);
        }
    }

    /**
     * Sets the inversion state of a specific motor in the group.
     * @param inverted True to invert the motor direction, false to use normal direction.
     * @param motorIndex The index of the motor in the motor group whose inversion state is to be set.
     */
    public void setMotorsInverted(boolean inverted, int motorIndex){
        controlledMotors[motorIndex].setInverted(inverted);
    }
    
    /**
     * Returns the power of the first motor in the group (all motors should be the same)
     * @return The current power level of the first motor in the group.
     */
    public double getMotorPower() {
        return controlledMotors[0].get();
    }

    /**
     * Returns the position of the first motor in the group (all motors should be the same)
     * @return The current position of the first motor in the group.
     */
    public double getMotorPosition() {
        return controlledMotors[0].getEncoder().getPosition();
    }

    /**
     * Sets the ramp rate for all motors in the group.
     * @param rate The ramp rate in seconds to go from 0 to full throttle.
     */
    public void setMotorsRampRate(double rate) {
        for (CANSparkBase motor : controlledMotors) {
            motor.setOpenLoopRampRate(rate);
            motor.setClosedLoopRampRate(rate);
        }
    }

    /**
     * Returns the average temperature of all motors in the group.
     * @return The average temperature of the motors in the group.
     */
    public double getMotorsAverageTemperature() {
        double totalTemp = 0.0;
        for (CANSparkBase motor : controlledMotors) {
            totalTemp += motor.getMotorTemperature();
        }
        return totalTemp / controlledMotors.length;
    }

    /**
     * Returns the average bus voltage of all motors in the group.
     * @return The average bus voltage of the motors in the group.
     */
    public double getMotorsAverageBusVoltage() {
        double totalVoltage = 0.0;
        for (CANSparkBase motor : controlledMotors) {
            totalVoltage += motor.getBusVoltage();
        }
        return totalVoltage / controlledMotors.length;
    }

    /**
     * Returns the average output current of all motors in the group.
     * @return The average output current of the motors in the group.
     */
    public double getMotorsAverageOutputCurrent() {
        double totalCurrent = 0.0;
        for (CANSparkBase motor : controlledMotors) {
            totalCurrent += motor.getOutputCurrent();
        }
        return totalCurrent / controlledMotors.length;
    }

    /**
     * Checks if any motor in the group has faults.
     * @return True if any motor in the group has faults, false otherwise.
     */
    public boolean checkMotorsForFaults() {
        for (CANSparkBase motor : controlledMotors) {
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
        return controlledMotors[0].getEncoder();
    }

}
