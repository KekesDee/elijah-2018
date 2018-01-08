package org.usfirst.frc.team1844.robot.commands.lift;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TimedLift extends Command {

	private double speed, time_lim;
	private Timer timer;
	
    public TimedLift(double speed, double time_lim) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires (Robot.m_liftarm);
    	timer = new Timer();
    	this.speed = speed;
    	this.time_lim = time_lim;
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.m_liftarm.setSpeed(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (timer.get() >= time_lim || Robot.m_liftarm.hit_bot() || Robot.m_liftarm.hit_top() );
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.m_liftarm.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
