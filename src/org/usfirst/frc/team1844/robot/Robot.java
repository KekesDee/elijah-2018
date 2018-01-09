/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1844.robot;

import org.usfirst.frc.team1844.robot.commands.autonomous.AutoScript2;
import org.usfirst.frc.team1844.robot.subsystems.Climber;
import org.usfirst.frc.team1844.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1844.robot.subsystems.Intake;
import org.usfirst.frc.team1844.robot.subsystems.LiftArm;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	public static OI m_oi;
	public static DriveTrain m_drivetrain;
	public static Intake m_intake;
	public static Climber m_climber;
	public static LiftArm m_liftArm;

	private Command m_autonomousCommand;
	private SendableChooser<RobotConstants.AutoPositions> m_positionChooser = new SendableChooser<>();
	private SendableChooser<RobotConstants.AutoOptions> m_autonomousChooser = new SendableChooser<>();

	private Timer m_prefsUpdateTimer = new Timer();
	private Preferences m_robotPrefs;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code
	 */
	@Override
	public void robotInit() {

		// Load all of the robot preferences from the NetworkTables,
		// and then repopulate them to ensure they are visible on the
		// SmartDashboard
		m_robotPrefs = Preferences.getInstance();
		RobotConstants.loadPrefs(m_robotPrefs);
		RobotConstants.repopulatePrefs(m_robotPrefs);

		m_drivetrain = new DriveTrain();
		m_liftArm = new LiftArm();
		m_intake = new Intake();
		m_climber = new Climber();

		m_oi = new OI();

		CameraServer.getInstance().startAutomaticCapture();

		m_positionChooser.addDefault("Left", RobotConstants.AutoPositions.kLeft);
		m_positionChooser.addObject("Middle", RobotConstants.AutoPositions.kMiddle);
		m_positionChooser.addObject("Right", RobotConstants.AutoPositions.kRight);
		SmartDashboard.putData("Auto Position", m_positionChooser);

		m_autonomousChooser.addDefault("Switch", RobotConstants.AutoOptions.kSwitch);
		m_autonomousChooser.addObject("Scale", RobotConstants.AutoOptions.kScale);
		m_autonomousChooser.addObject("Breach", RobotConstants.AutoOptions.kBreach);
		m_autonomousChooser.addObject("Nothing", RobotConstants.AutoOptions.kNothing);
		SmartDashboard.putData("Auto options", m_autonomousChooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
		m_drivetrain.resetPID();
		m_drivetrain.resetEncoders();

		m_prefsUpdateTimer.reset();
		m_prefsUpdateTimer.start();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();

		// Not really necessary. But why not.
		if (m_prefsUpdateTimer.hasPeriodPassed(3))
			RobotConstants.updatePrefs(m_robotPrefs);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_drivetrain.resetEncoders();
		RobotConstants.updatePrefs(m_robotPrefs);
		m_drivetrain.updateTunings();

		m_autonomousCommand = new AutoScript2(m_positionChooser.getSelected(), m_autonomousChooser.getSelected());

		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		m_drivetrain.resetEncoders();
		RobotConstants.updatePrefs(m_robotPrefs);
		m_drivetrain.updateTunings();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	private void updateSmartDashboard() {
		SmartDashboard.putNumber("Left Enc Raw", m_drivetrain.getLeftEncRaw());
		SmartDashboard.putNumber("Right Enc Raw", m_drivetrain.getRightEncRaw());
		SmartDashboard.putNumber("Left Enc Dist", m_drivetrain.getLeftEncDist());
		SmartDashboard.putNumber("Right Enc Dist", m_drivetrain.getRightEncDist());
	}
}
