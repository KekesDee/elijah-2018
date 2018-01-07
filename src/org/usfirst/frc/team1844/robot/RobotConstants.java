package org.usfirst.frc.team1844.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;

/**
 * The RobotConstants is a collection of all of the other constants used
 * throughout the project. Here, all of the constants other than port numbers
 * are specified. For example, {@link Joystick} ports, buttons, and axis are
 * *not* what this class is meant for. Rather, it is used for constants like
 * P,I, and D tunings, constants used in calculations such as gravity, wheel
 * diameter, etc.
 */
public class RobotConstants {

	public static final int ENCODER_PPR = 128; // Pulses Per Revolution
	public static final double WHEEL_DIA = 3.97; // 4" Coulsons
	public static final double GEARBOX_REDUCTION = 1;
	public static final double ENCODER_DISTANCE_PER_PULSE = WHEEL_DIA * Math.PI / ENCODER_PPR * GEARBOX_REDUCTION;

	public static double exampleValue;

	/**
	 * Load all of the preferences from the file saved on the roboRIO. These
	 * preferences are editable using the RobotPreferences widget on the
	 * SmartDashboard, and can be changed without having to recompile and reupload
	 * code to the robot.
	 * 
	 * @param prefs the {@link Preferences} to read the date from
	 */
	public static void loadPrefs(Preferences prefs) {
		exampleValue = prefs.getDouble("Example", 0.0);
		
	}

	/**
	 * Re-upload all of the robot preferences to the RobotPreferences widget on the
	 * SmartDashboard. This is important because if an entry is deleted on the
	 * SmartDashboard, it will not reappear unless the user can recreate it with the
	 * exact same key, or it is programmatically repopulated with a default value,
	 * as we are doing here.
	 * 
	 * @param prefs
	 *            the {@link Preferences} to write the values to
	 */
	public static void repopulatePrefs(Preferences prefs) {
		prefs.putDouble("Example", exampleValue);
	}

}
