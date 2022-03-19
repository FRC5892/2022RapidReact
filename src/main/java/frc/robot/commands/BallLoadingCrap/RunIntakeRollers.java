// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ballLoadingCrap;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.serializer.Intake;

public class RunIntakeRollers extends CommandBase {
	private Intake intake;

	/** Creates a new RunIntakeRollers. */
	public RunIntakeRollers(Intake i) {
		// Use addRequirements() here to declare subsystem dependencies.
		intake = i;
		addRequirements(intake);
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
			// outake
			intake.setMotors(OperatorInput.driverJoystick.getLeftTriggerAxis() * Constants.INTAKE_SPEED_MULTIPLIER);
		}
		else if (OperatorInput.driverJoystick.getRightTriggerAxis() > 0) {
			// intake
			intake.setMotors(-OperatorInput.driverJoystick.getRightTriggerAxis() * Constants.INTAKE_SPEED_MULTIPLIER);
		}
		else {
			intake.stopMotors();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
