// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
	/** Creates a new PhotonVision. */
	static PhotonCamera camera = new PhotonCamera("photonvision");
	static PIDController turnController = new PIDController(0.1, 0, 0.0);
	DriveTrain driveTrain;

	public PhotonVision() {
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void rotateToTarget() {
		var result = camera.getLatestResult();

		if (result.hasTargets()) {
			double rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
			driveTrain.driveWithJoysticks(0, rotationSpeed);
		}
	}
}
