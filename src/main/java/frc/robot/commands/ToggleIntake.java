// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends CommandBase {
	private Intake intake;
	private boolean finished;

	/** Creates a new ToggleIntake. */
	public ToggleIntake(Intake i) {
		intake = i;
		finished = false;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		finished = false;
		if (intake.primarySolenoidPosition() == Value.kForward) {
			intake.setPositionSolenoids(Value.kReverse);
		}
		intake.togglePositionSolenoids();
		finished = true;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finished;
	}
}
