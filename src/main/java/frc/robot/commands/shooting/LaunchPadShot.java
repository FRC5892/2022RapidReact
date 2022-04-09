// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PolynomialFunction;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.serializer.Accumulator;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.TurretVision;
import frc.robot.commands.shooting.LinearInterpolation;

public class LaunchPadShot extends CommandBase {
	private Flywheel flywheel;
	private Turret turret;
	private Hood hood;
	private TurretVision turretVision;
	private Accumulator accumulator;
	private Tower tower;
	private Kicker kicker;
	private DriveTrain driveTrain;
	private PIDController driveTrainPIDController = new PIDController(Constants.DRIVETRAIN_AIM_PID_CONSTANTS[0],
			Constants.DRIVETRAIN_AIM_PID_CONSTANTS[1], Constants.DRIVETRAIN_AIM_PID_CONSTANTS[2]);

	private boolean shootWhenReady;

	/** Creates a new AimAndShoot. */
	public LaunchPadShot(Flywheel f, Turret t, Hood h, Accumulator a, Tower tw, Kicker k, TurretVision tv,
			DriveTrain dt) {
		flywheel = f;
		turret = t;
		hood = h;
		accumulator = a;
		tower = tw;
		kicker = k;
		turretVision = tv;
		driveTrain = dt;
		driveTrainPIDController.setTolerance(.25);
		addRequirements(flywheel, turret, hood, accumulator, tower, kicker, turretVision, driveTrain);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		shootWhenReady = false;
		flywheel.setSetpoint(2700);
		hood.setSetpoint(40);
		hood.enable();
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		//if (turretVision.hasTargets()) {
			if (shootWhenReady) {
				if (hood.atSetpoint() && flywheel.atSetpoint() /*&& turret.atSetpoint()*/) {
					// shoot
					kicker.setMotors(Constants.KICKER_SHOOT_SPEED);
					tower.setMotors(Constants.TOWER_SPEED);
				}
			} else {
				driveTrain.arcadeDrive(0, -driveTrainPIDController.calculate(turretVision.xAngle(), -3));

				if (driveTrainPIDController.atSetpoint()) {
					driveTrain.stopMotors();
					shootWhenReady = true;
				}
			}
		//}

		// preload balls
		if (!shootWhenReady) {
			if (!kicker.hasBall()) {
				kicker.setMotors(Constants.KICKER_SPEED);
				tower.setMotors(Constants.TOWER_SPEED);

			}
			else if (!tower.hasBall()) {
				accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
				tower.setMotors(Constants.TOWER_SPEED);
			}

			if (kicker.hasBall()) {
				kicker.stopMotors();

			} 
			
			else {
				kicker.setMotors(Constants.KICKER_SPEED);
				tower.setMotors(Constants.TOWER_SPEED);
			}
			if (tower.hasBall() && kicker.hasBall()) {
				tower.stopMotors();
			}
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		//flywheel.stop();
		// turret.stop();
		hood.stopMotors();
		tower.stopMotors();
		kicker.stopMotors();
		driveTrain.stopMotors();
		System.out.println("Stopping");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
