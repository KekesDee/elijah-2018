package org.usfirst.frc.team1844.robot.commands.intake;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Command for intaking and ejecting cubes
 */
public class RunIntake extends Command {

	private double m_speed;

	public RunIntake(double speed) {
		requires(Robot.m_intake);
		m_speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.m_intake.set(m_speed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_intake.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
