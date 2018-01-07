package org.usfirst.frc.team1844.robot.commands;

import org.usfirst.frc.team1844.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Wait extends Command {

	public Wait() {
		requires(Robot.m_climber);
		requires(Robot.m_drivetrain);
		requires(Robot.m_intake);
		requires(Robot.m_liftarm);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

}
