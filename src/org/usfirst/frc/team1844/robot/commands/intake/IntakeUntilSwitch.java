package org.usfirst.frc.team1844.robot.commands.intake;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Intake cubes until a limit switch is hit
 */
public class IntakeUntilSwitch extends Command {
	private double m_speed;

	public IntakeUntilSwitch(double speed) {
		requires(Robot.m_intake);
		m_speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.m_intake.getHasCube())
			Robot.m_intake.setSpeed(0);
		else
			Robot.m_intake.setSpeed(m_speed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.m_intake.getHasCube();
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
