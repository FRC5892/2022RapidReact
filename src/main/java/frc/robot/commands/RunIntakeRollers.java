// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Kicker;

public class RunIntakeRollers extends CommandBase {
	private Intake intake;
	private Accumulator accumulator;
	private Tower tower;
	private Kicker kicker;
	private Timer timer = new Timer();

	/** Creates a new RunIntakeRollers. */
	public RunIntakeRollers(Intake i, Accumulator a, Tower t, Kicker k) {
		// Use addRequirements() here to declare subsystem dependencies.
		intake = i;
		accumulator = a;
		tower = t;
		kicker = k;
		addRequirements(intake, accumulator, tower, kicker);
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
			// outake
			intake.setMotors(OperatorInput.driverJoystick.getLeftTriggerAxis() * Constants.INTAKE_SPEED_MULTIPLIER);
			accumulator.setMotors(
					OperatorInput.driverJoystick.getLeftTriggerAxis() * Constants.ACCUMULATOR_SPEED_MULTIPLIER);
		}
		else if (OperatorInput.driverJoystick.getRightTriggerAxis() > 0) {
			// intake
			intake.setMotors(-OperatorInput.driverJoystick.getRightTriggerAxis() * Constants.INTAKE_SPEED_MULTIPLIER);
			timer.reset();
			timer.start();
		}
		else {
			intake.stopMotors();
		}
		if (timer.get() < Constants.PRELOAD_TIMEOUT) {
			if (!kicker.hasBall()) {
				kicker.setMotors(Constants.KICKER_SPEED);
			}
			else {
				kicker.stopMotors();
			}
			if (!tower.hasBall()) {
				tower.setMotors(Constants.TOWER_SPEED);
			}
			else {
				tower.stopMotors();
			}
			if (!tower.hasBall() || !kicker.hasBall()) {
				accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
			}
			else {
				accumulator.stopMotors();
			}
		}
		// if timer (if kicker no ball (run kicker), if kicker no ball or tower no ball (run accumulator))
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.stopMotors();
		accumulator.stopMotors();
		tower.stopMotors();
		kicker.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
