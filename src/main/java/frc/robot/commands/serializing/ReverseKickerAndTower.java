// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.serializing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;

public class ReverseKickerAndTower extends CommandBase {
	private Kicker kicker;
	private Tower tower;

	/** Creates a new RunKickerandTower. */
	public ReverseKickerAndTower(Kicker k, Tower t) {
		// Use addRequirements() here to declare subsystem dependencies.
		kicker = k;
		tower = t;
		addRequirements(kicker, tower);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		kicker.setMotors(-.5);
		tower.setMotors(-.5);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
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
