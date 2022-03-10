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

// !IMPORTANT! pairs of motor ports stored in an array are always ordered left motor right motor
public final class Constants {

	// Constants such as camera and target height stored. Change per robot and goal!
	final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
	final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
	// Angle between horizontal and the camera.
	final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

	// How far from the target we want to be
	final double GOAL_RANGE_METERS = Units.feetToMeters(3);

	public final static double ACUTAL_WHEEL_DIAMETER = Units.inchesToMeters(4.1966);
	public final static double ENCODER_CONVERSION_FACTOR = (Math.PI * ACUTAL_WHEEL_DIAMETER) / 1024;

	public static final int[] FLYWHEEL_ENCODER_PORTS = new int[] { 4, 5 };
	public static final double FLYWHEEL_ENCODER_CONVERSION_FACTOR = 1/.4;
	public static final int[] FLYWHEEL_MOTOR_IDS = new int[] { 9, 10 };
	public static final double FLYWHEEL_SHOOTING_SPEED = 0;
	public static final double[] FLYWHEEL_PID_CONSTANTS = new double[] {0.00125/.4, 0.002/.4, 0/.4};

	public static final int TURRET_POTENTIOMETER = 6;
	public static final double TURRET_POTENTIOMETER_CONVERSION_FACTOR = 1;

	public static final int TURRET_MOTOR_ID = 16;

	public static final double TURRET_SCAN_SPEED = 0.3;

	public static final double TURRETVISION_CAMERA_HEIGHT = 0;
	public static final double TURRETVISION_CAMERA_PITCH = 0;
	public static final double GOAL_HEIGHT = 0;

	public static final int HOOD_POTENTIOMETER_PORT = 7;
	public static final double HOOD_ENCODER_CONVERSION_FACTOR = 1;

	public static final int HOOD_MOTOR_ID = 8;

	public static final int[] INTAKE_MOTOR_PORTS = new int[] { 11, 12 };
	public static final double INTAKE_SPEED_MULTIPLIER = 1;
	public static final int[] INTAKE_LEFT_PRIMARY_SOLENOID_PORTS = new int[] { 1, 2 };
	public static final int[] INTAKE_RIGHT_PRIMARY_SOLENOID_PORTS = new int[] { 3, 4 };
	public static final int[] INTAKE_LEFT_POSITION_SOLENOID_PORTS = new int[] { 5, 6 };
	public static final int[] INTAKE_RIGHT_POSITION_SOLENOID_PORTS = new int[] { 7, 8 };

	public static final int ACCUMULATOR_MOTOR_PORT = 13;
	public static final double ACCUMULATOR_SPEED = 0.3;
	public static final double ACCUMULATOR_SPEED_MULTIPLIER = 1;
	public static final int KICKER_SENSOR_PORT = 12;
	public static final double KICKER_SPEED = 0;

	public static final String LIMELIGHT_NAME = "limelight";
	public static final double TOWER_SPEED = 0;
	public static final int KICKER_MOTOR_PORT = 15;
	public static final int TOWER_MOTOR_PORT = 14;
}
