// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.Accumulator;

public class RunAccumulator extends CommandBase {
	private Accumulator accumulator;

	/** Creates a new RunAccumulator. */
	public RunAccumulator(Accumulator a) {
		accumulator = a;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(accumulator);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// default
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (OperatorInput.driverJoystick.getLeftTriggerAxis() > 0) {
			// spit ball out
			accumulator.setMotors(
					OperatorInput.driverJoystick.getLeftTriggerAxis() * Constants.ACCUMULATOR_SPEED_MULTIPLIER);
		}
		if (OperatorInput.driverJoystick.getRightTriggerAxis() > 0) {
			accumulator.setMotors(-OperatorInput.driverJoystick.getRightTriggerAxis());
		}
		else {
			accumulator.stopMotors();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		accumulator.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
