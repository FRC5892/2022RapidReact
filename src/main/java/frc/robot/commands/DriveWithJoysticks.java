// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.DriveTrain;

public class DriveWithJoysticks extends CommandBase {
	private DriveTrain driveTrain;
	private double xSpeed;
	private double zRotation;

	/** Creates a new DriveWithJoysticks. */
	public DriveWithJoysticks(DriveTrain dt) {
		// Use addRequirements() here to declare subsystem dependencies.
		driveTrain = dt;
		addRequirements(driveTrain);
		SmartDashboard.putNumber("Joystick Deadzone", Constants.JOYSTICK_DEADZONE);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		xSpeed = 0;
		zRotation = 0;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double joystickDeadzone = SmartDashboard.getNumber("Joystick Deadzone", Constants.JOYSTICK_DEADZONE);
		if (OperatorInput.driverJoystick.getLeftY() > joystickDeadzone
				|| OperatorInput.driverJoystick.getLeftY() < -joystickDeadzone) {
			xSpeed = OperatorInput.driverJoystick.getLeftY();
		}
		else {
			xSpeed = 0;
		}
		if (OperatorInput.driverJoystick.getRightX() > joystickDeadzone
				|| OperatorInput.driverJoystick.getRightX() < -joystickDeadzone) {
			zRotation = -OperatorInput.driverJoystick.getRightX();
		}
		else {
			zRotation = 0;
		}
		driveTrain.driveWithJoysticks(xSpeed, zRotation);
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
