package org.usfirst.frc.team1844.robot.commands.autonomous;

import org.usfirst.frc.team1844.robot.RobotConstants;
import org.usfirst.frc.team1844.robot.RobotConstants.AutoOptions;
import org.usfirst.frc.team1844.robot.RobotConstants.AutoPositions;
import org.usfirst.frc.team1844.robot.commands.driveTrain.AutoStraightDrive;
import org.usfirst.frc.team1844.robot.commands.driveTrain.TurnToAngle;
import org.usfirst.frc.team1844.robot.commands.intake.RunIntake;
import org.usfirst.frc.team1844.robot.commands.lift.ControlledLift;
import org.usfirst.frc.team1844.robot.commands.lift.TimedLift;
import org.usfirst.frc.team1844.robot.subsystems.LiftArm.ArmPositions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoScript extends CommandGroup {

	char[] gameSpecificMessage;
	
		
    public AutoScript(RobotConstants.AutoPositions pos, RobotConstants.AutoOptions objective) {

    	gameSpecificMessage = DriverStation.getInstance().getGameSpecificMessage().toCharArray(); // "LRL"
    	
    	//start position left
    	if (pos == AutoPositions.kLeft)
    	{
    		//scale right side is ours
    		if (objective == AutoOptions.kScale && gameSpecificMessage[1] == 'R')
    		{
    			addSequential (new AutoStraightDrive(95), 2);
    			addSequential (new TurnToAngle(90), 1.5);
    			addSequential (new AutoStraightDrive(20), 1);
    			addSequential (new TurnToAngle(-90), 1.5);
    			addSequential (new AutoStraightDrive(125), 3);
    			addSequential (new TurnToAngle(-90), 1.5);
    			addSequential (new AutoStraightDrive(200), 3);
    			addSequential (new TurnToAngle(90), 1.5);
    			addSequential (new AutoStraightDrive(105), 2);
    			addSequential (new TurnToAngle(-90), 1.5);
    			addParallel (new TimedLift (0.5, 2));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		
    		//scale left side is ours
    		else if (objective == AutoOptions.kScale && gameSpecificMessage[2] == 'L')
    		{
    			addSequential (new AutoStraightDrive(95), 2);
    			addSequential (new TurnToAngle(90), 1.5);
    			addSequential (new AutoStraightDrive(20), 1);
    			addSequential (new TurnToAngle(-90), 1.5);
    			addSequential (new AutoStraightDrive(230), 4);
    			addSequential (new TurnToAngle(90), 1.5);
    			addParallel (new TimedLift (0.5, 2));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		
    		//switch left side is ours
    		else if (objective == AutoOptions.kSwitch && gameSpecificMessage[2] == 'L')
    		{
    			addSequential (new AutoStraightDrive(95), 2);
    			addSequential (new TurnToAngle(-15), 2);
    			addParallel (new TimedLift (0.5, 0.3));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		
    		//switch right side is ours
    		else if (objective == AutoOptions.kSwitch && 
    				gameSpecificMessage[1] == 'R')
    		{
    			addSequential (new AutoStraightDrive(60), 2);
    			addSequential (new TurnToAngle(-90), 1);
    			addSequential (new AutoStraightDrive(135), 2);
    			addSequential (new TurnToAngle(90), 1);
    			addSequential (new AutoStraightDrive(39), 1.5);
    			addParallel (new TimedLift (0.5, 0.3));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		else
    		{
    			addSequential (new AutoStraightDrive (100), 3);
    		}
    	}
    	
    	////////////////////////////////////////////////////////////////////////////
    	
    	
    	//start position right
    	if (pos == AutoPositions.kRight)
    	{
    		//scale left side is ours
    		if (objective == AutoOptions.kScale && gameSpecificMessage[2] == 'L')
    		{
    			addSequential (new AutoStraightDrive(100), 2);
    			addSequential (new TurnToAngle(-90), 1.5);
    			addSequential (new AutoStraightDrive(80), 1);
    			addSequential (new TurnToAngle(90), 1.5);
    			addSequential (new AutoStraightDrive(125), 4);
    			addSequential (new TurnToAngle(90), 1.5);
    			addSequential (new AutoStraightDrive(240), 2);
    			addSequential (new TurnToAngle(-90), 1.5);
    			addSequential (new AutoStraightDrive(100), 2);
    			addSequential (new TurnToAngle(90), 1.5);
    			addParallel (new TimedLift (0.5, 0.3));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		
    		//scale right side is ours
    		else if (objective == AutoOptions.kScale && gameSpecificMessage[2] == 'R')
    		{
    			addSequential (new AutoStraightDrive(100), 2);
    			addSequential (new TurnToAngle(-90), 1.5);
    			addSequential (new AutoStraightDrive(80), 1);
    			addSequential (new TurnToAngle(90), 1.5);
    			addSequential (new AutoStraightDrive(225), 4);
    			addSequential (new TurnToAngle(90), 1.5);
    			addSequential (new AutoStraightDrive(40), 2);
    			addSequential (new TurnToAngle(180), 1.5);
    			addParallel (new TimedLift (0.5, 2));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		
    		//switch left side is ours
    		else if (objective == AutoOptions.kSwitch && gameSpecificMessage[1] == 'L')
    		{
    			addSequential (new AutoStraightDrive(50), 2);
    			addSequential (new TurnToAngle(90), 1);
    			addSequential (new AutoStraightDrive(115), 2);
    			addSequential (new TurnToAngle(-90), 1);
    			addSequential (new AutoStraightDrive(50), 1.5);
    			addParallel (new TimedLift (0.5, 0.3));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		// switch right side is ours
    		else if (objective == AutoOptions.kSwitch && gameSpecificMessage[1] == 'R')
    		{
    			addSequential (new AutoStraightDrive(100), 2);
    			addParallel (new TimedLift (0.5, 0.3));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		else
    		{
    			addSequential (new AutoStraightDrive (100), 3);
    		}
    	}
    	
    	
    	/////////////////////////////////////////////////////////////////////////////
    	
    	//start position middle
    	if (pos == AutoPositions.kMiddle)
    	{
    		//scale left side is ours
    		if (objective == AutoOptions.kScale && gameSpecificMessage[2] == 'L')
    		{
    			addSequential (new AutoStraightDrive(36), 2);
    			addSequential (new TurnToAngle(90), 1.5);
    			addSequential (new AutoStraightDrive(135), 1);
    			addSequential (new TurnToAngle(-90), 1.5);
    			addSequential (new AutoStraightDrive(290), 4);
    			addSequential (new TurnToAngle(90), 1.5);
    			addSequential (new AutoStraightDrive(-30), 1);
    			addParallel (new ControlledLift (0.5, 2, ArmPositions.kScalePos));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		
    		//scale right side is ours
    		else if (objective == AutoOptions.kScale && gameSpecificMessage[2] == 'R')
    		{
    			addSequential (new AutoStraightDrive(36), 2);
    			addSequential (new TurnToAngle(-90), 1.5);
    			addSequential (new AutoStraightDrive(130), 1);
    			addSequential (new TurnToAngle(90), 1.5);
    			addSequential (new AutoStraightDrive(290), 4);
    			addSequential (new TurnToAngle(-90), 1.5);
    			addSequential (new AutoStraightDrive(-35), 1);
    			addParallel (new TimedLift (0.5, 2));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		
    		//switch right side is ours
    		else if (objective == AutoOptions.kSwitch && gameSpecificMessage[1] == 'R')
    		{
    			addSequential (new AutoStraightDrive(36), 2);
    			addSequential (new TurnToAngle(-90), 1);
    			addSequential (new AutoStraightDrive(55), 2);
    			addSequential (new TurnToAngle(90), 1);
    			addSequential (new AutoStraightDrive(60), 1.5);
    			addParallel (new TimedLift (0.5, 0.3));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		
    		//switch left side is ours
    		else if (objective == AutoOptions.kSwitch && gameSpecificMessage[1] == 'L')
    		{
    			addSequential (new AutoStraightDrive(36), 2);
    			addSequential (new TurnToAngle(90), 1);
    			addSequential (new AutoStraightDrive(60), 2);
    			addSequential (new TurnToAngle(-90), 1);
    			addSequential (new AutoStraightDrive(60), 1.5);
    			addParallel (new TimedLift (0.5, 0.3));
    			addSequential (new RunIntake(-1), 0.7);
    		}
    		else
    		{
    			addSequential (new AutoStraightDrive (100), 3);
    		}
    	}
    }
}
