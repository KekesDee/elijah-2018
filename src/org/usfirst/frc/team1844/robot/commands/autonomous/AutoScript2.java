package org.usfirst.frc.team1844.robot.commands.autonomous;

import org.usfirst.frc.team1844.robot.RobotConstants.AutoOptions;
import org.usfirst.frc.team1844.robot.RobotConstants.AutoPositions;
import org.usfirst.frc.team1844.robot.commands.driveTrain.AutoStraightDrive;
import org.usfirst.frc.team1844.robot.commands.driveTrain.TurnToAngle;
import org.usfirst.frc.team1844.robot.commands.intake.RunIntake;
import org.usfirst.frc.team1844.robot.commands.lift.TimedLift;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoScript2 extends CommandGroup {

	public AutoScript2(AutoPositions pos, AutoOptions objective) {

		// If we want to do nothing, just stop.
		if (objective == AutoOptions.kNothing)
			return;

		// If we want to breach, drive forwards 96in
		if (objective == AutoOptions.kBreach) {
			addSequential(new AutoStraightDrive(96), 5);
			return;
		}

		double turn90Inwards = 90;
		if (getHot(objective) == 'R')
			turn90Inwards *= -1;

		// If we are in the middle...
		if (pos == AutoPositions.kMiddle) {
			addSequential(new AutoStraightDrive(56), 2);
			addSequential(new TurnToAngle(-turn90Inwards), 2);

			if (objective == AutoOptions.kSwitch) {
				if (getHot(objective) == 'L')
					addSequential(new AutoStraightDrive(56 + 28), 2);
				else
					addSequential(new AutoStraightDrive(56), 2);

				addParallel(liftToSwitch());
				addSequential(new TurnToAngle(turn90Inwards), 2);
				addSequential(new AutoStraightDrive(44), 2);
				ejectCube();
			}

			if (objective == AutoOptions.kScale) {
				addSequential(new AutoStraightDrive(88), 2);
				addSequential(new TurnToAngle(-turn90Inwards), 2);
				addSequential(new AutoStraightDrive(-194), 4);
				addParallel(liftToScale());
				addSequential(new TurnToAngle(25 * turn90Inwards / Math.abs(turn90Inwards)), 1);
				ejectCube();
			}

		}

		// If we are on the 'hot' side...
		else if (isHot(pos, objective)) {

			if (objective == AutoOptions.kScale) {
				addSequential(new AutoStraightDrive(310), 5);
				addParallel(liftToScale());
				addSequential(new TurnToAngle(-turn90Inwards), 2);
				// Drive back??
				ejectCube();
			}

			else if (objective == AutoOptions.kSwitch) {
				addSequential(new AutoStraightDrive(154), 3);
				addParallel(liftToSwitch());
				addSequential(new TurnToAngle(turn90Inwards), 2);
				addSequential(new AutoStraightDrive(17), 2);
				ejectCube();
			}
		}

		// If we are NOT on the 'hot' side...
		else {
			addSequential(new AutoStraightDrive(226), 5);
			addSequential(new TurnToAngle(turn90Inwards), 2);

			if (objective == AutoOptions.kScale) {
				addSequential(new AutoStraightDrive(200), 5);
				addSequential(new TurnToAngle(turn90Inwards), 2);
				addSequential(new AutoStraightDrive(-42), 2);
				addParallel(liftToScale());
				ejectCube();
			}

			else if (objective == AutoOptions.kSwitch) {
				addSequential(new AutoStraightDrive(175), 4);
				addParallel(liftToSwitch());
				addSequential(new TurnToAngle(turn90Inwards), 2);
				addSequential(new AutoStraightDrive(9), 2);
				ejectCube();
			}
		}
	}

	private boolean isHot(AutoPositions pos, AutoOptions objective) {
		char[] data = DriverStation.getInstance().getGameSpecificMessage().toCharArray(); // "LRL"

		return data[objective.toInt()] == pos.toChar();
	}

	private char getHot(AutoOptions objective) {
		char[] data = DriverStation.getInstance().getGameSpecificMessage().toCharArray(); // "LRL"

		return data[objective.toInt()];
	}

	private Command liftToScale() {
		return new TimedLift(0.5, 2);
	}

	private Command liftToSwitch() {
		return new TimedLift(0.5, 0.3);
	}

	private void ejectCube() {
		addSequential(new RunIntake(-1), 2);
	}
}
