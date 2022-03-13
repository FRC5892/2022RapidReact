// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Tower;

public class RunKickerandTower extends CommandBase {
	private Kicker kicker;
	private Tower tower;

	/** Creates a new RunKickerandTower. */
	public RunKickerandTower(Kicker k, Tower t) {
		// Use addRequirements() here to declare subsystem dependencies.
		kicker = k;
		tower = t;
		addRequirements(kicker, tower);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		kicker.setMotors(-.5);
		tower.setMotors(-.5);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		kicker.stopMotors();
		tower.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
