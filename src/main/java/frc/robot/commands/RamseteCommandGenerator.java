// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class RamseteCommandGenerator {
	private static Trajectory trajectory;

	public static SequentialCommandGroup generate(DriveTrain driveTrain, String pathName) {
		// try {
		// trajectory = TrajectoryUtil
		// .fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON));
		// }
		// catch (IOException e) {
		// e.printStackTrace();
		// }
		trajectory = PathPlanner.loadPath(pathName, 8, 5);

		// var autoVoltageConstraint =
		// new DifferentialDriveVoltageConstraint(
		// new SimpleMotorFeedforward(
		// Constants.drive.ksVolts,
		// Constants.drive.kvVoltSecondsPerMeter,
		// Constants.drive.kaVoltSecondsSquaredPerMeter),
		// Constants.drive.kDriveKinematics,
		// 10);

		// // Create config for trajectory
		// TrajectoryConfig config =
		// new TrajectoryConfig(
		// Constants.auton.kMaxSpeedMetersPerSecond,
		// Constants.auton.kMaxAccelerationMetersPerSecondSquared)
		// // Add kinematics to ensure max speed is actually obeyed
		// .setKinematics(Constants.drive.kDriveKinematics)
		// // Apply the voltage constraint
		// .addConstraint(autoVoltageConstraint);

		RamseteCommand ramseteCommand = new RamseteCommand(trajectory, driveTrain::getPose,
				new RamseteController(Constants.auton.kRamseteB, Constants.auton.kRamseteZeta),
				new SimpleMotorFeedforward(Constants.drive.ksVolts, Constants.drive.kvVoltSecondsPerMeter,
						Constants.drive.kaVoltSecondsSquaredPerMeter),
				new DifferentialDriveKinematics(Constants.drive.kTrackwidth), driveTrain::getWheelSpeeds,
				new PIDController(Constants.drive.kP, 0, 0), new PIDController(Constants.drive.kP, 0, 0),
				// RamseteCommand passes volts to the callback
				driveTrain::tankDriveVolts, driveTrain);

		// Reset odometry to the starting pose of the trajectory.
		driveTrain.resetOdometry(trajectory.getInitialPose());

		// Run path following command, then stop at the end.
		return ramseteCommand.andThen(() -> driveTrain.stopMotors());
	}
}
