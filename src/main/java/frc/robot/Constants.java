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

	public final static double ACUTAL_WHEEL_DIAMETER = Units.inchesToMeters(4.1966);
	public final static double ENCODER_CONVERSION_FACTOR = (Math.PI * ACUTAL_WHEEL_DIAMETER) / 1024;

	public static final int[] FLYWHEEL_ENCODER_PORTS = new int[] { 4, 5 };
	public static final double FLYWHEEL_ENCODER_CONVERSION_FACTOR = 1 / .4;
	public static final int[] FLYWHEEL_MOTOR_IDS = new int[] { 9, 10 };
	public static final double FLYWHEEL_SHOOTING_SPEED = 2100;
	public static final double FLYWHEEL_SHOOTING_ANGLE = 27.5;
    public static final double[] FLYWHEEL_SPARKMAX_PIDF = new double[] { 0.00105, 5.5E-7, 1E-7, 0.00001 };

	public static final double FLYWHEEL_LONG_SHOOTING_SPEED = 2700;
	public static final double FLYWHEEL_LONG_SHOOTING_ANGLE = 40;

	public static final int TURRET_POTENTIOMETER = 6;
	public static final double TURRET_POTENTIOMETER_CONVERSION_FACTOR = 1;
	public static final int TURRET_MOTOR_ID = 16;
	public static final double TURRET_SCAN_SPEED = 0.3;
	public static final double TURRETVISION_CAMERA_HEIGHT = 32+15/16;
	public static final double TURRETVISION_CAMERA_PITCH = 35;
	public static final double GOAL_HEIGHT = 104;

	public static final int HOOD_POTENTIOMETER_PORT = 0;
	public static final double HOOD_ENCODER_CONVERSION_FACTOR = 1;
	public static final int HOOD_MOTOR_ID = 8;
	public static final double[] HOOD_PID = new double[] { 0.1, 0, 0 };

	public static final int INTAKE_MOTOR_PORT = 7;
	public static final double INTAKE_SPEED_MULTIPLIER = 1;
	public static final int[] INTAKE_PISTON_SOLENOID_PORTS = new int[] { 7, 6 };

	public static final int ACCUMULATOR_MOTOR_PORT = 12;
	public static final double ACCUMULATOR_SPEED = 1;
	public static final double ACCUMULATOR_SPEED_MULTIPLIER = 1;
	public static final int KICKER_SENSOR_PORT = 8;
	public static final double KICKER_SPEED = .25;
	public static final double KICKER_SHOOT_SPEED = .5;
	// public static final double KICKER_SHOOT_SPEED = .25;

	public static final double TOWER_SPEED = .5;
	public static final int TOWER_MOTOR_PORT = 13;
	public static final int TOWER_SENSOR_PORT = 9;

	public static final double PRELOAD_TIMEOUT = 3;

	public static final double[] DRIVETRAIN_AIM_PID_CONSTANTS = new double[] { 0.1, 0, 0.015 };
	public static final String LIMELIGHT_NAME = "limelight";
	public static final int KICKER_MOTOR_PORT = 15;
	public static final double AUTONOMOUS_SPEED = .5;
	public static final double AUTONOMOUS_SHOOT_TIMER = 5;

	public static final int[] CLIMB_MOTOR_PORTS = new int[] { 19, 20 };
	public static final int[] CLIMB_ACTUATION_SOLENOID_PORTS = new int[] { 0, 1 };
	public static double AUTON_DISTANCE = 2; //meters
}
