package org.usfirst.frc.team1844.robot.commands.climb;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClimbWithJoystick extends Command {

	public ClimbWithJoystick() {
		requires(Robot.m_climber);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double speed = Robot.m_oi.getDriverJoystick().getTriggerAxis(Hand.kRight) // R Trigger = in
				- Robot.m_oi.getDriverJoystick().getTriggerAxis(Hand.kLeft); // L Trigger = out

		Robot.m_climber.set(speed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_climber.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
