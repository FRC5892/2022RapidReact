// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Shooter.Flywheel;

public class timedShoot extends CommandBase {
	private Flywheel flywheel;
	private boolean finished;
	private Accumulator accumulator;
	private Tower tower;
	private Kicker kicker;
	private boolean shoot;
	private double timerValue;
	private Timer timer;

	/** Creates a new AimAndShoot. */
	public timedShoot(Flywheel f, Accumulator a, Tower tw, Kicker k, double timerValue) {
		flywheel = f;
		accumulator = a;
		tower = tw;
		kicker = k;

		addRequirements(flywheel, accumulator, tower, kicker);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.setSetpoint(Constants.FLYWHEEL_SHOOTING_SPEED);
		flywheel.enable();
		shoot = false;
		timer.start();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (flywheel.atSetpoint()) {
			kicker.setMotors(Constants.KICKER_SPEED);
			accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
		}
		else {
			if (!kicker.hasBall()) {
				kicker.setMotors(Constants.KICKER_SPEED);
				tower.setMotors(Constants.TOWER_SPEED);
				accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
			}
			else if (!tower.hasBall()) {
				accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
				tower.setMotors(Constants.TOWER_SPEED);
			}
			if (kicker.hasBall()) {
				kicker.stopMotors();
			}
			if (kicker.hasBall() && tower.hasBall()) {
				kicker.stopMotors();
				accumulator.stopMotors();
				tower.stopMotors();
			}
		}
		finished = (timer.get() > timerValue);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.stop();
		accumulator.stopMotors();
		tower.stopMotors();
		kicker.stopMotors();
		timer.reset();
		shoot = false;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finished;
	}
}
