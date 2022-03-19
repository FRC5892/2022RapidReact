// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.serializer.Accumulator;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;

public class PreloadBall extends CommandBase {
	private boolean finished;
	private Accumulator accumulator;
	private Tower tower;
	private Kicker kicker;

	/** Creates a new AimAndShoot. */
	public PreloadBall(Accumulator a, Tower tw, Kicker k) {
		accumulator = a;
		tower = tw;
		kicker = k;
		addRequirements(accumulator, tower, kicker);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		//
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (!kicker.hasBall()) {
			kicker.setMotors(Constants.KICKER_SPEED);
			tower.setMotors(Constants.TOWER_SPEED);
			accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
		}
		else {
			end(true);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		accumulator.stopMotors();
		tower.stopMotors();
		kicker.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finished;
	}
}
