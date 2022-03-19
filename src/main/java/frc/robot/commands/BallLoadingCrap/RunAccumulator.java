// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ballLoadingCrap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.serializer.Accumulator;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;

public class RunAccumulator extends CommandBase {
	private Accumulator accumulator;
	private Kicker kicker;
	private Timer timer;
	private Tower tower;
	private boolean preloading;

	/** Creates a new RunAccumulator. */
	public RunAccumulator(Accumulator a, Kicker k, Tower t) {
		accumulator = a;
		kicker = k;
		tower = t;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(accumulator);
		timer = new Timer();
		preloading = false;
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
			preloading = false;
			accumulator.setMotors(
					-OperatorInput.driverJoystick.getLeftTriggerAxis() * Constants.ACCUMULATOR_SPEED_MULTIPLIER);
		}
		else if (OperatorInput.driverJoystick.getLeftTriggerAxis() == 0 && !preloading) {
			accumulator.stopMotors();
		}
		if (OperatorInput.driverJoystick.getRightTriggerAxis() > 0) {
			// intake
			preloading = true;
			timer.reset();
			timer.start();
			accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
		}
		if (((timer.get() > Constants.PRELOAD_TIMEOUT || (kicker.hasBall() && tower.hasBall())) && preloading)) {
			preloading = false;
			accumulator.stopMotors();
			timer.stop();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		preloading = false;
		accumulator.stopMotors();
		timer.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
