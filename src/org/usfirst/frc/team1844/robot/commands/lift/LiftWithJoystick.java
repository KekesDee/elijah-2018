package org.usfirst.frc.team1844.robot.commands.lift;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LiftWithJoystick extends Command {

	public LiftWithJoystick() {
		requires(Robot.m_liftArm);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double speed = Math.pow(Robot.m_oi.getDriverJoystick().getY(Hand.kRight), 3); // Right Y

		if (speed < 0)
			speed *= 0.4;
		else
			speed *= 0.15;

		Robot.m_liftArm.set(speed - 0.05);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
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
