// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.serializer.Kicker;

public class RunKickerTest extends CommandBase {
	private Kicker kicker;

	/** Creates a new RunKickerTest. */
	public RunKickerTest(Kicker k) {
		kicker = k;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(kicker);
		// SmartDashboard.putNumber("Kicker Set Speed", 0);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// default
		kicker.setMotors(0.5);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		kicker.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
