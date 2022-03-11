// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BallLoadingCrap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Tower;

public class RunAccumulator extends CommandBase {
	private Accumulator accumulator;
	private Kicker kicker;
	private Timer timer;
	private Tower tower;

	/** Creates a new RunAccumulator. */
	public RunAccumulator(Accumulator a, Kicker k, Tower t) {
		accumulator = a;
		kicker = k;
		tower = t;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(accumulator);
		timer = new Timer();
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer.reset();
		// default
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		
		if (OperatorInput.driverJoystick.getLeftTriggerAxis() > 0) {
			System.out.println(OperatorInput.driverJoystick.getLeftTriggerAxis());
			// spit ball out
			accumulator.setMotors(
					-OperatorInput.driverJoystick.getLeftTriggerAxis() * Constants.ACCUMULATOR_SPEED_MULTIPLIER);
		}
		if (OperatorInput.driverJoystick.getRightTriggerAxis() > 0) {
			// System.out.println(OperatorInput.driverJoystick.getRightTriggerAxis());
			// intake
			timer.reset();
			timer.start();
			System.out.println(timer.get());
			// accumulator.setMotors(OperatorInput.driverJoystick.getRightTriggerAxis());
			// driver shouldn't have control of accumulator
		}
		if (timer.get() < Constants.PRELOAD_TIMEOUT && (!kicker.hasBall() || !tower.hasBall())) {
			System.out.println("stuff");
			accumulator.setMotors(Constants.ACCUMULATOR_SPEED);

		}
		else {
			accumulator.stopMotors();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		accumulator.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
