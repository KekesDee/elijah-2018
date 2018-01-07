package org.usfirst.frc.team1844.robot.commands.driveTrain;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Turn the robot the specified angle, in degrees
 */
public class TurnToAngle extends Command {

	private double m_targetAngle;

	/**
	 * @param angle The angle to turn, in degrees
	 */
	public TurnToAngle(double angle) {
		requires(Robot.m_drivetrain);
		m_targetAngle = angle;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.m_drivetrain.setTargetAngle(m_targetAngle);
		Robot.m_drivetrain.enablePID();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.m_drivetrain.getPIDOnTarget();
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_drivetrain.disablePID();
		Robot.m_drivetrain.stop();
		Robot.m_drivetrain.resetPID();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
