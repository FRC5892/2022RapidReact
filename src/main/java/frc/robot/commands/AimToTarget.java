// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.OperatorInput;
import frc.robot.subsystems.PhotonVision;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AimToTarget extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final PhotonVision photonVision;

	public AimToTarget(PhotonVision photon) {
		photonVision = photon;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(photonVision);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		boolean aim = OperatorInput.driverJoystick.getAButton();
		if (aim) {
			photonVision.rotateToTarget();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
