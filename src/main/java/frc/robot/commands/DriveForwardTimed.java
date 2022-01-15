// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveForwardTimed extends CommandBase {
	DriveTrain driveTrain;
	private boolean finish = false;

	/**
	 * Creates a new DriveForwardTimed.
	 *
	 * @param dt
	 */
	public DriveForwardTimed(DriveTrain dt) {
		driveTrain = dt;
		addRequirements(driveTrain);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (driveTrain.getLeftPosition() < Constants.DRIVE_FORWARD_ROTATIONS) {
			driveTrain.driveForward(Constants.AUTONOMOUS_SPEED);
		}
		else {
			finish = true;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		driveTrain.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finish;
	}
}
