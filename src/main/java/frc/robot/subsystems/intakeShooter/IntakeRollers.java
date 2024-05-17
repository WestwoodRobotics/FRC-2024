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

    private CANSparkMax upperIntakeRollerMotor; // Renamed for clarity
    private CANSparkMax lowerRollerMotor;

    private PIDController upperRollerPIDController;
    private PIDController lowerRollerPIDController;

    private CANSparkMax stowMotor;

    private IntakeShooterPositions intakeShooterPosition;
    @SuppressWarnings("unused")
    private boolean isRollerPIDControl;

    /**
     * Constructs the IntakeRollers subsystem with specific motor ports.
     * Initializes the motors, PID controllers, and sets the initial shooter position.
     */
    public IntakeRollers() {
        upperIntakeRollerMotor = new CANSparkMax(IntakeShooterConstants.kUpperMotorPort, MotorType.kBrushless);
        lowerRollerMotor = new CANSparkMax(IntakeShooterConstants.kLowerMotorPort, MotorType.kBrushless);
        stowMotor = new CANSparkMax(IntakeShooterConstants.kStowMotorPort, MotorType.kBrushless);
        
        upperRollerPIDController = new PIDController(IntakeShooterConstants.kUpperRollerP, 
        IntakeShooterConstants.kUpperRollerI, IntakeShooterConstants.kUpperRollerD);
        lowerRollerPIDController = new PIDController(IntakeShooterConstants.kLowerRollerP, 
        IntakeShooterConstants.kLowerRollerI, IntakeShooterConstants.kLowerRollerD);
        intakeShooterPosition = IntakeShooterPositions.HOME;
    }

    /**
     * Sets the power for both the upper and lower intake roller motors.
     * @param power The power to set, where 1 is full forward and -1 is full reverse.
     */
    public void setRollerPower(double power) {
        isRollerPIDControl = false;
        upperIntakeRollerMotor.set(power);
        lowerRollerMotor.set(-power);
    }

    /**
     * Sets the RPM for both the upper and lower intake roller motors using PID control.
     * @param upperRollerRPM The target RPM for the upper roller motor.
     * @param lowerRollerRPM The target RPM for the lower roller motor.
     */
    public void setRollerRPM(double upperRollerRPM, double lowerRollerRPM) {
        isRollerPIDControl = true;
        upperRollerPIDController.setSetpoint(upperRollerRPM);
        lowerRollerPIDController.setSetpoint(lowerRollerRPM);
    }

    /**
     * Gets the current RPM of the upper intake roller motor.
     * @return The current RPM of the upper intake roller motor.
     */
    public double getUpperRPM() {
        return upperIntakeRollerMotor.getEncoder().getVelocity();
    }

    /**
     * Gets the current RPM of the lower roller motor.
     * @return The current RPM of the lower roller motor.
     */
    public double getLowerRPM() {
        return lowerRollerMotor.getEncoder().getVelocity();
    }
    
    /**
     * Sets the power for the stow motor.
     * @param power The power to set, where 1 is full forward and -1 is full reverse.
     */
    public void setStowPower(double power) {
        isRollerPIDControl = false;
        stowMotor.set(power);
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
        upperIntakeRollerMotor.set(0);
        lowerRollerMotor.set(0);
        stowMotor.set(0);
    }

    /**
     * Stops the roller motors.
     */
    public void stopRollerMotor() {
        upperIntakeRollerMotor.set(0);
        lowerRollerMotor.set(0);
    }

    /**
     * Stops the stow motor.
     */
    public void stopStowMotor() {
        stowMotor.set(0);
    }

    /**
     * Gets the current state of the IntakeRollers subsystem.
     * @return The current state of the IntakeRollers subsystem.
     */
    public IntakeShooterPositions getState() {
        return intakeShooterPosition;
    }

    /**
     * Sets the state of the IntakeRollers subsystem.
     * @param position The new state for the IntakeRollers subsystem.
     */
    public void setPositionState(IntakeShooterPositions position) {
        intakeShooterPosition = position;
    }

    /**
     * Periodically updates the subsystem's state and displays it on the SmartDashboard.
     */
    @Override
    public void periodic() {
        SmartDashboard.putString("Intake Shooter State", intakeShooterPosition.toString()); 
        SmartDashboard.putNumber("Upper Roller RPM", upperIntakeRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Lower Roller RPM", lowerRollerMotor.getEncoder().getVelocity());
    }
}
