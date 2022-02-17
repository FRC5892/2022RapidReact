// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {
	// Constants such as camera and target height stored. Change per robot and goal!
	final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
	final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
	// Angle between horizontal and the camera.
	final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

	// How far from the target we want to be
	final double GOAL_RANGE_METERS = Units.feetToMeters(3);

	public final static double ACUTAL_WHEEL_DIAMETER = 0.104672061299478;
	public final static double ENCODER_CONVERSION_FACTOR = (Math.PI * ACUTAL_WHEEL_DIAMETER) / 1024;
	// public final static double ENCODER_CONVERSION_FACTOR = 1 / 1024;
	public final static double DRIVE_GEAR_RATIO = 4.44;

	public final static double JOYSTICK_DEADZONE = 0.01;

	public final static double[] LEFT_DRIVE_PID_CONSTANTS = new double[] { 0, 0, 0 };
	public final static double[] RIGHT_DRIVE_PID_CONSTANTS = new double[] { 0, 0, 0 };

}
