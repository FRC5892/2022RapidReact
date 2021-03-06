// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretVision extends SubsystemBase {

	/** Creates a new TurretVision. */
	public TurretVision() {
		// template
	}

	public boolean hasTargets() {
		return NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("tv").getDouble(0) > 0;
	}

	public double distanceFromTarget() {
		double height = (Constants.GOAL_HEIGHT - Constants.TURRETVISION_CAMERA_HEIGHT);
		double limelightPitch = NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("ty").getDouble(0);
		double angle = Units.degreesToRadians(Constants.TURRETVISION_CAMERA_PITCH + limelightPitch);

		return (height)/Math.tan(angle);

		// return height / Math
		// 		.tan(Units.degreesToRadians(Constants.TURRETVISION_CAMERA_PITCH) + NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("ty").getDouble(0));
	}

	public double xAngle() {
		return NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("tx").getDouble(0);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Target Distance", distanceFromTarget());
		// This method will be called once per scheduler run
	}
}
