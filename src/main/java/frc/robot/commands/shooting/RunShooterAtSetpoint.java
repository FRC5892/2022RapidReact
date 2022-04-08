// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Flywheel;

public class RunShooterAtSetpoint extends CommandBase {
	private Flywheel flywheel;

	/** Creates a new RunShooterAtSetpoint. */
	public RunShooterAtSetpoint(Flywheel f) {
		// Use addRequirements() here to declare subsystem dependencies.
		flywheel = f;
		addRequirements(flywheel);
		// SmartDashboard.putNumber("Flywheel Setpoint RPM", 0);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.setSetpoint(Constants.FLYWHEEL_SHOOTING_SPEED);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// default
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
