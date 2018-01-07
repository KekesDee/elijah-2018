package org.usfirst.frc.team1844.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	SpeedController motor1, motor2;
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void climb(double speed)
    {
    	motor1.set(speed);
    	motor2.set(speed);
    }
    
    public void stop()
    {
    	motor1.stopMotor();
    	motor2.stopMotor();
    }
}

