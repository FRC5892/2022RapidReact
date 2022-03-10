// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.PreloadBall;
import frc.robot.commands.RunAccumulator;
import frc.robot.commands.OutputFlywheelEncoder;
import frc.robot.commands.RunFlywheelFullSpeed;
import frc.robot.commands.RunIntakeRollers;
import frc.robot.commands.RunKickerTest;
import frc.robot.commands.RunKickerandTower;
import frc.robot.commands.RunShooterAtSetpoint;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.Turret;
import frc.robot.subsystems.Shooter.TurretVision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...

	private DriveWithJoysticks driveWithJoysticks;

	private DriveTrain driveTrain;

	private Flywheel flywheel;

	private RunFlywheelFullSpeed runFlywheelFullSpeed;

	private Intake intake;

	private RunIntakeRollers runIntakeRollers;

	private ToggleIntake toggleIntake;

	private Accumulator accumulator;

	private RunAccumulator runAccumulator;

	private Tower tower;

	private Kicker kicker;

	private Hood hood;

	private Turret turret;

	private AimAndShoot aimAndShoot;

	private TurretVision turretVision;

	private PreloadBall preloadBall;

	private RunShooterAtSetpoint runShooterAtSetpoint;

	private RunKickerTest runKickerTest;

	private OutputFlywheelEncoder outputFlywheelEncoder;

	private RunKickerandTower runKickerAndTower;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		driveTrain = new DriveTrain();
		driveWithJoysticks = new DriveWithJoysticks(driveTrain);
		driveTrain.setDefaultCommand(driveWithJoysticks);

		flywheel = new Flywheel();
		outputFlywheelEncoder = new OutputFlywheelEncoder(flywheel);
		flywheel.setDefaultCommand(outputFlywheelEncoder);
		runFlywheelFullSpeed = new RunFlywheelFullSpeed(flywheel);
		runShooterAtSetpoint = new RunShooterAtSetpoint(flywheel);

		accumulator = new Accumulator();
		runAccumulator = new RunAccumulator(accumulator);
		accumulator.setDefaultCommand(runAccumulator);

		intake = new Intake();
		runIntakeRollers = new RunIntakeRollers(intake);
		intake.setDefaultCommand(runIntakeRollers);
		toggleIntake = new ToggleIntake(intake);

		tower = new Tower();
		kicker = new Kicker();
		turret = new Turret();
		hood = new Hood();
		turretVision = new TurretVision();

		preloadBall = new PreloadBall(accumulator, tower, kicker);

		runKickerTest = new RunKickerTest(kicker);

		aimAndShoot = new AimAndShoot(flywheel, turret, hood, accumulator, tower, kicker, turretVision);
		runKickerAndTower = new RunKickerandTower(kicker, tower);
		// Configure the button bindingsz
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by instantiating a
	 * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
	 * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		OperatorInput.runFlywheelFullButton.whileHeld(runFlywheelFullSpeed);
		OperatorInput.toggleIntake.whenPressed(toggleIntake);
		OperatorInput.toggleIntakePosition.whenPressed(new InstantCommand(intake::togglePositionSolenoids, intake));
		OperatorInput.toggleAimAndShoot.whenPressed(aimAndShoot);
		OperatorInput.toggleRunShooterAtSetpoint.whileHeld(runShooterAtSetpoint);
		OperatorInput.holdRunKickerTest.whileHeld(runKickerTest);
		OperatorInput.toggleIntakePosition.whenPressed(new InstantCommand(intake::togglePositionSolenoids, intake));
		OperatorInput.runKickerAndTower.whileHeld(runKickerAndTower);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	// public Command getAutonomousCommand() {
	// // An ExampleCommand will run in autonomous
	// return m_autoCommand;
	// }
}
