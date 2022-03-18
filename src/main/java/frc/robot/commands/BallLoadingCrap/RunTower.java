// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ballLoadingCrap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.accumulator.Kicker;
import frc.robot.subsystems.accumulator.Tower;

public class RunTower extends CommandBase {
	private Kicker kicker;
	private Timer timer;
	private Tower tower;

	/** Creates a new RunAccumulator. */
	public RunTower(Kicker k, Tower t) {
		kicker = k;
		tower = t;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(tower);
		timer = new Timer();
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// default
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (OperatorInput.driverJoystick.getRightTriggerAxis() > 0) {
			// intake
			timer.reset();
			timer.start();
			tower.setMotors(Constants.TOWER_SPEED);
		}
		if (timer.get() > Constants.PRELOAD_TIMEOUT || (kicker.hasBall() && tower.hasBall())) {
			tower.stopMotors();
			timer.stop();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		tower.stopMotors();
		timer.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
