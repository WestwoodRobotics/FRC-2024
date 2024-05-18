package frc.robot.subsystems.intakeShooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * The IntakeRollers subsystem controls the rollers used for intaking and shooting balls.
 * It consists of upper and lower rollers, each controlled by a CANSparkMax motor controller.
 */
public class IntakeRollers extends SubsystemBase {

    private CANSparkMax upperIntakeRollerMotorController; // Renamed for clarity
    private CANSparkMax lowerIntakeRollerMotorController;

    private PIDController upperIntakeRollerPIDController;
    private PIDController lowerIntakeRollerPIDController;

    private CANSparkMax intakeStowMotorController;

    private IntakeShooterPositions intakeRollerPosition;
    @SuppressWarnings("unused")
    private boolean isRollerPIDControl;

    /**
     * Constructs the IntakeRollers subsystem with specific motor ports.
     * Initializes the motors, PID controllers, and sets the initial shooter position.
     */
    public IntakeRollers() {
        upperIntakeRollerMotorController = new CANSparkMax(IntakeShooterConstants.kUpperMotorPort, MotorType.kBrushless);
        lowerIntakeRollerMotorController = new CANSparkMax(IntakeShooterConstants.kLowerMotorPort, MotorType.kBrushless);
        intakeStowMotorController = new CANSparkMax(IntakeShooterConstants.kStowMotorPort, MotorType.kBrushless);
        
        upperIntakeRollerPIDController = new PIDController(IntakeShooterConstants.kUpperRollerP, 
        IntakeShooterConstants.kUpperRollerI, IntakeShooterConstants.kUpperRollerD);
        lowerIntakeRollerPIDController = new PIDController(IntakeShooterConstants.kLowerRollerP, 
        IntakeShooterConstants.kLowerRollerI, IntakeShooterConstants.kLowerRollerD);
        intakeRollerPosition = IntakeShooterPositions.HOME;
    }

    /**
     * Sets the power for both the upper and lower intake roller motors.
     * @param power The power to set, where 1 is full forward and -1 is full reverse.
     */
    public void setRollerPower(double power) {
        isRollerPIDControl = false;
        upperIntakeRollerMotorController.set(power);
        lowerIntakeRollerMotorController.set(-power);
    }

    /**
     * Sets the RPM for both the upper and lower intake roller motors using PID control.
     * @param upperRollerRPM The target RPM for the upper roller motor.
     * @param lowerRollerRPM The target RPM for the lower roller motor.
     */
    public void setRollerRPM(double upperRollerRPM, double lowerRollerRPM) {
        isRollerPIDControl = true;
        upperIntakeRollerPIDController.setSetpoint(upperRollerRPM);
        lowerIntakeRollerPIDController.setSetpoint(lowerRollerRPM);
    }

    /**
     * Gets the current RPM of the upper intake roller motor.
     * @return The current RPM of the upper intake roller motor.
     */
    public double getUpperRPM() {
        return upperIntakeRollerMotorController.getEncoder().getVelocity();
    }

    /**
     * Gets the current RPM of the lower roller motor.
     * @return The current RPM of the lower roller motor.
     */
    public double getLowerRPM() {
        return lowerIntakeRollerMotorController.getEncoder().getVelocity();
    }
    
    /**
     * Sets the power for the stow motor.
     * @param power The power to set, where 1 is full forward and -1 is full reverse.
     */
    public void setStowPower(double power) {
        isRollerPIDControl = false;
        intakeStowMotorController.set(power);
    }

    /**
     * Stops all motors in the IntakeRollers subsystem.
     */
    public void stopAllMotors() {
        stopRollerMotor();
        stopStowMotor();
    }

    /**
     * Stops all intake and shooter roller motors.
     */
    public void stopAllIntakeShooterRollers() {
        upperIntakeRollerMotorController.set(0);
        lowerIntakeRollerMotorController.set(0);
        intakeStowMotorController.set(0);
    }

    /**
     * Stops the roller motors.
     */
    public void stopRollerMotor() {
        upperIntakeRollerMotorController.set(0);
        lowerIntakeRollerMotorController.set(0);
    }

    /**
     * Stops the stow motor.
     */
    public void stopStowMotor() {
        intakeStowMotorController.set(0);
    }

    /**
     * Gets the current state of the IntakeRollers subsystem.
     * @return The current state of the IntakeRollers subsystem.
     */
    public IntakeShooterPositions getState() {
        return intakeRollerPosition;
    }

    /**
     * Sets the state of the IntakeRollers subsystem.
     * @param position The new state for the IntakeRollers subsystem.
     */
    public void setPositionState(IntakeShooterPositions position) {
        intakeRollerPosition = position;
    }

    /**
     * Periodically updates the subsystem's state and displays it on the SmartDashboard.
     */
    @Override
    public void periodic() {
        SmartDashboard.putString("Intake Shooter State", intakeRollerPosition.toString()); 
        SmartDashboard.putNumber("Upper Roller RPM", upperIntakeRollerMotorController.getEncoder().getVelocity());
        SmartDashboard.putNumber("Lower Roller RPM", lowerIntakeRollerMotorController.getEncoder().getVelocity());
    }
}
