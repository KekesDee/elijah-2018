package org.usfirst.frc.team1844.robot.commands;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;

/**
 *Command for releasing cubes
 */
public class IntakeOutWithY extends Command {

	private static final double SPEED = -0.3;
	
    public IntakeOutWithY() {
    	requires(Robot.m_intake);
    }


    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.m_intake.setMotorSpeed(SPEED);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.m_intake.stopMotors();
    }
    
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}