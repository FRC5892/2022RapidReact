// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretVision extends SubsystemBase {
	double heightFromLimelightToGoal;
	NetworkTableEntry targets;
	NetworkTableEntry horizontalOffsetAngle;
	NetworkTableEntry verticalOffsetAngle;

	/** Creates a new TurretVision. */
	public TurretVision() {
		NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME);
		targets = limelightTable.getEntry("tv");
		horizontalOffsetAngle = limelightTable.getEntry("tx");
		verticalOffsetAngle = limelightTable.getEntry("ty");
		heightFromLimelightToGoal = Constants.GOAL_HEIGHT - Constants.TURRETVISION_CAMERA_HEIGHT;
	}

	public boolean hasTargets() {
		return targets.getDouble(0) > 0;
	}

	public double distanceFromTarget() {
		double radians = Units.degreesToRadians(Constants.TURRETVISION_CAMERA_PITCH + verticalOffsetAngle.getDouble(0));
		return heightFromLimelightToGoal * Math.tan(radians);
	}

	public double xAngle() {
		return horizontalOffsetAngle.getDouble(0);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Target Distance", distanceFromTarget());
		// This method will be called once per scheduler run
	}
}
