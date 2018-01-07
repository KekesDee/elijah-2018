package org.usfirst.frc.team1844.robot.commands.autonomous;

import org.usfirst.frc.team1844.robot.RobotConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoScript extends CommandGroup {

	
    public AutoScript(RobotConstants.AutoPositions pos, RobotConstants.AutoOptions objective) {


    DriverStation.getInstance().getGameSpecificMessage(); // "LRL"
    	
    	
    }
}
