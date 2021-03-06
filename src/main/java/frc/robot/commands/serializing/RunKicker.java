// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.serializing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;

public class RunKicker extends CommandBase {
	private Kicker kicker;
	private Timer timer;

	/** Creates a new Run. */
	public RunKicker(Kicker k, Tower t) {
		kicker = k;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(kicker);
		timer = new Timer();
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("running kicker");
		// default
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (OperatorInput.driverJoystick.getRightTriggerAxis() > 0.02) {
			System.out.println("Trigger hit");
			// intake
			timer.reset();
			timer.start();
			kicker.setMotors(Constants.KICKER_SPEED);
		}
		if (timer.get() > Constants.PRELOAD_TIMEOUT || kicker.hasBall()) {
			kicker.stopMotors();
			timer.stop();
		}
		System.out.println(timer.get());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		kicker.stopMotors();
		System.out.println("Kicker ended");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
