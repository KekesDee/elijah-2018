package org.usfirst.frc.team1844.robot.commands.lift;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TimedLift extends Command {

	private double m_speed, m_timeLim;
	private Timer m_timer = new Timer();

	public TimedLift(double speed, double time_lim) {
		requires(Robot.m_liftArm);
		m_speed = speed;
		m_timeLim = time_lim;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.m_liftArm.set(m_speed);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return m_timer.get() >= m_timeLim;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_liftArm.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
