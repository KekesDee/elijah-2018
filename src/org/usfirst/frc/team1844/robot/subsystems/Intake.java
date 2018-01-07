package org.usfirst.frc.team1844.robot.subsystems;

import org.usfirst.frc.team1844.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {

    SpeedController leftIntake;
    SpeedController rightIntake;
    SpeedController topIntake;
    
    DigitalInput limit;

    public void initDefaultCommand() {
        leftIntake = new WPI_TalonSRX(RobotMap.IntakeLeft);
        rightIntake = new WPI_TalonSRX(RobotMap.IntakeRight);
        topIntake = new WPI_TalonSRX(RobotMap.IntakeTop);
    }
    
    public void setMotorSpeed (double speed)
    {
    	leftIntake.set(speed);
    	rightIntake.set(speed);
    	topIntake.set(speed);
    	
    	while (limit.get());
    		stopMotors();
    	
    }
    
    public void stopMotors()
    {
    	leftIntake.stopMotor();
    	rightIntake.stopMotor();
    	topIntake.stopMotor();
    }
        
}

