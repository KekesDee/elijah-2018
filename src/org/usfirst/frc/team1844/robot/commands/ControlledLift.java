package org.usfirst.frc.team1844.robot.commands;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/*
  this should be used to lift a specific distance 
  since we are not using encoders for now, uses timer
 */
public class ControlledLift extends Command {

	Timer mytimer;
	double time_lim;
	double speed;
	
    public ControlledLift(double speed) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.liftarm);
    	mytimer = new Timer();
    	time_lim = 0;
    	this.speed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	mytimer.start();
    	Robot.liftarm.set_speed(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return mytimer.get() >= time_lim;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.liftarm.stopMotor();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
