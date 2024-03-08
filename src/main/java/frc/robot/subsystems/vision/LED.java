package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
	
	Spark blinkin;

	public LED(int channel) {
		blinkin = new Spark(channel);
	}

	public void setLights(double value) {
		blinkin.set(value);
	}

	
	
	
}