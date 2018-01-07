package org.usfirst.frc.team1844.robot.commands.lift;

import org.usfirst.frc.team1844.robot.Robot;
import org.usfirst.frc.team1844.robot.subsystems.LiftArm.ArmPositions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/*
  this should be used to lift a specific distance 
  since we are not using encoders for now, uses timer
 */
public class ControlledLift extends Command {

	Timer mytimer;
	ArmPositions target;
	boolean reach_pos;
	
	double time_lim;
	double speed;
	
    public ControlledLift(double speed, double time_lim, ArmPositions target) {

    	requires(Robot.m_liftarm);
    	mytimer = new Timer();
    	reach_pos = false;
    	this.time_lim = time_lim;
    	this.speed = speed;
    	this.target = target;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	mytimer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	reach_pos = Robot.m_liftarm.lift_to_pos(target, speed);
    }
    
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (mytimer.get() >= time_lim || reach_pos);
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
