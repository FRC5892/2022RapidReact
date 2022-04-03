// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Flywheel;

public class OutputFlywheelEncoder extends CommandBase {
	private Flywheel flywheel;

	/** Creates a new OutputFlywheelEncoder. */
	public OutputFlywheelEncoder(Flywheel f) {
		// Use addRequirements() here to declare subsystem dependencies.
		flywheel = f;
		addRequirements(flywheel);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// flywheel.setMotors(.2);
		flywheel.stop();
		// flywheel.setSetpoint(100);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		SmartDashboard.putNumber("Flywheel Velocity", flywheel.getVelocity());

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
