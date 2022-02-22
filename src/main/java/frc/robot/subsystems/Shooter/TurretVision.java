// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretVision extends SubsystemBase {
	private PhotonCamera camera = new PhotonCamera("turretVision");
	private PhotonPipelineResult result;

	/** Creates a new TurretVision. */
	public TurretVision() {
	}

	public boolean hasTargets() {
		return result.hasTargets();
	}

	public double distanceFromTarget() {
		return PhotonUtils.calculateDistanceToTargetMeters(Constants.TURRETVISION_CAMERA_HEIGHT, Constants.GOAL_HEIGHT,
				Constants.TURRETVISION_CAMERA_PITCH, Units.degreesToRadians(result.getBestTarget().getPitch()));
	}

	public double targetYaw() {
		return result.getBestTarget().getYaw();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		result = camera.getLatestResult();
	}
}
