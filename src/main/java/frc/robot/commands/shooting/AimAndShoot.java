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

public class AimAndShoot extends CommandBase {
	private Flywheel flywheel;
	private Turret turret;
	private Hood hood;
	private TurretVision turretVision;
	private boolean finished;
	private double[] hoodRangingCoefficients = new double[] { .00000000000001, .00000000000001, .00000000000001 };
	private double[] flywheelRangingCoefficients = new double[] { .00000000000001, .00000000000001, .00000000000001 };
	private Accumulator accumulator;
	private Tower tower;
	private Kicker kicker;
	private DriveTrain driveTrain;
	private PIDController driveTrainPIDController = new PIDController(Constants.DRIVETRAIN_AIM_PID_CONSTANTS[0],
			Constants.DRIVETRAIN_AIM_PID_CONSTANTS[1], Constants.DRIVETRAIN_AIM_PID_CONSTANTS[2]);

	private boolean shootWhenReady;
	private double[] xCoords;
	private double[] shooterYCoords;
	private double[] hoodYCoords;

	/** Creates a new AimAndShoot. */
	public AimAndShoot(Flywheel f, Turret t, Hood h, Accumulator a, Tower tw, Kicker k, TurretVision tv,
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
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (turretVision.hasTargets()) {
			// turret.setSetpoint(turret.getMeasurement() - turretVision.xAngle());
			xCoords = new double[] {0,71,74,78,81.5,84,85.7,90,94,98,102.5,106.2,111,115,120,124,128,133,137,141.5,147,151,154,159,164.5,170,175,180,184,188,192,195,200,204,208,211,215.7,221,225,228};
			shooterYCoords = new double[] {2160,2160,
				2190,
				2190,
				2190,
				2190,2240,2290,2290,2320,2345,2365,2385,2410,2440,2460,2480,2480,2495,2515,2580,2580,2610,2610,2650,2680,2660,2700,2680,2720,2720,2720,2720,2760,2780,2810,2830,2850,2850,2860};
			hoodYCoords = new double[] {32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,33,33,33,33,33,33,33,33,34,34,35,35,35,36,36,37,37,37,37,37,38,38};

			if (shootWhenReady) {
				if (hood.atSetpoint() && flywheel.atSetpoint() /*&& turret.atSetpoint()*/) {
					// shoot
					kicker.setMotors(Constants.KICKER_SHOOT_SPEED);
					accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
				}
			} else {
				driveTrain.arcadeDrive(0, -driveTrainPIDController.calculate(turretVision.xAngle(), -3));

				if (driveTrainPIDController.atSetpoint()) {
					driveTrain.stop();
					flywheel.setSetpoint(LinearInterpolation.calculate(xCoords, shooterYCoords,  turretVision.distanceFromTarget()));
					hood.setSetpoint((double) LinearInterpolation.calculate(xCoords, hoodYCoords,  turretVision.distanceFromTarget()));
					hood.enable();
					shootWhenReady = true;
				}
			}
		}

		// preload balls
		if (!shootWhenReady) {
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

				if (tower.hasBall()) {
					accumulator.stopMotors();
					tower.stopMotors();
				}
			} 
			
			else {
				kicker.setMotors(Constants.KICKER_SPEED);
				tower.setMotors(Constants.TOWER_SPEED);
				accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
			}
		}
		// else {
		// if (turret.atLeftLimit()) {
		// reverse = true;
		// }
		// else if (turret.atRightLimit()) {
		// reverse = false;
		// }
		// if (reverse) {
		// turret.setMotor(-(Constants.TURRET_SCAN_SPEED));
		// }
		// else {
		// turret.setMotor(Constants.TURRET_SCAN_SPEED);
		// }
		// }

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.stop();
		// turret.stop();
		hood.stop();
		accumulator.stopMotors();
		tower.stopMotors();
		kicker.stopMotors();
		driveTrain.stop();
		System.out.println("Stopping");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finished;
	}
}
