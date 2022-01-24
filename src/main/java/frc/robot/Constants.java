// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
	public final class drive {

		public static final double ksVolts = 1.83;
		public static final double kvVoltSecondsPerMeter = 0.245;
		public static final double kaVoltSecondsSquaredPerMeter = 0.444;
		public static final double kTrackwidth = 0.6;
		public static final double kP = 0.944;
	}

	public final class auton {

		public static final double kMaxSpeedMetersPerSecond = 18;
		public static final double kMaxAccelerationMetersPerSecondSquared = 7;
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;

	}
}
