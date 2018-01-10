package org.usfirst.frc.team1844.robot.commands.driveTrain;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Drive the robot with a joystick
 */
public class TeleopDrive extends Command {

	public TeleopDrive() {
		requires(Robot.m_drivetrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when thijs Command is scheduled to run
	protected void execute() {
		double drive = Math.pow(Robot.m_oi.getDriverJoystick().getY(Hand.kLeft), 3);
		double curve = Math.pow(Robot.m_oi.getDriverJoystick().getX(Hand.kLeft), 3);
		
		curve *= 0.7;
		
		Robot.m_drivetrain.drive(drive, curve);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_drivetrain.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
