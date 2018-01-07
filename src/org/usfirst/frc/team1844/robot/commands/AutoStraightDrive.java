package org.usfirst.frc.team1844.robot.commands;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoStraightDrive extends Command {
	
	double dist;
	double timeout;

    public AutoStraightDrive(double dist, double timeout) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.dist = dist;
    	this.timeout = timeout;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(timeout);
    	Robot.m_drivetrain.setdist(dist);
    	Robot.m_drivetrain.PIDDrive();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.m_drivetrain.PIDStop();
    	Robot.m_drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
