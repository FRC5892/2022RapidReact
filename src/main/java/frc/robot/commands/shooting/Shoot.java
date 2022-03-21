// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.serializer.Accumulator;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;
import frc.robot.subsystems.shooter.Flywheel;

public class Shoot extends CommandBase {
	private Flywheel flywheel;
	private Accumulator accumulator;
	private Tower tower;
	private Kicker kicker;
	private Hood hood;
	private Double speed;
	private Double angle;

	/** Creates a new AimAndShoot. */
	public Shoot(Flywheel f, Accumulator a, Tower tw, Kicker k, Hood h, Double s, Double ag) {
		flywheel = f;
		accumulator = a;
		tower = tw;
		kicker = k;
		hood = h;
		speed = s;
		angle = ag;

		addRequirements(flywheel, accumulator, tower, kicker, hood);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.setSetpoint(speed);
		flywheel.enable();
		hood.setSetpoint(angle);
		hood.enable();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (flywheel.atSetpoint() && hood.atSetpoint()) {
			kicker.setMotors(Constants.KICKER_SHOOT_SPEED);
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


	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.stop();
		accumulator.stopMotors();
		tower.stopMotors();
		kicker.stopMotors();
		hood.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
