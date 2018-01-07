package org.usfirst.frc.team1844.robot.commands;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Donuts extends Command {

	Timer mytimer;
	
    public Donuts() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	mytimer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	 mytimer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drivetrain.spin();
    }

    protected boolean isFinished() {
        return mytimer.get()>5;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
