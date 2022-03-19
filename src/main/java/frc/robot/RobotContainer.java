// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.OutputFlywheelEncoder;
import frc.robot.commands.RunClimb;
import frc.robot.commands.RunFlywheelFullSpeed;
import frc.robot.commands.RunKickerTest;
import frc.robot.commands.RunKickerandTower;
import frc.robot.commands.RunShooterAtSetpoint;
import frc.robot.commands.Shoot;
import frc.robot.commands.timedShoot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.serializer.Accumulator;
import frc.robot.subsystems.serializer.Intake;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.TurretVision;
import frc.robot.commands.autonomous.SimpleAuton;
import frc.robot.commands.ballLoadingCrap.RunAccumulator;
import frc.robot.commands.ballLoadingCrap.RunIntakeRollers;
import frc.robot.commands.ballLoadingCrap.RunKicker;
import frc.robot.commands.ballLoadingCrap.RunTower;

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

	private Accumulator accumulator;

	private RunAccumulator runAccumulator;
	private Tower tower;
	private Kicker kicker;
	private Hood hood;
	private Turret turret;

	private AimAndShoot aimAndShoot;
	private TurretVision turretVision;

	private RunShooterAtSetpoint runShooterAtSetpoint;
	private RunKickerandTower runKickerAndTower;
	private RunKickerTest runKickerTest;

	private OutputFlywheelEncoder outputFlywheelEncoder;

	private SimpleAuton simpleAuto;

	private RunKicker runKicker;

	private RunTower runTower;

	private Compressor compressor;

	private Shoot shoot;

	private timedShoot TimedShoot; 
	
	private Climb climb;

	private RunClimb runClimb;
	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {

		compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
		compressor.enableDigital();

		driveTrain = new DriveTrain();
		driveWithJoysticks = new DriveWithJoysticks(driveTrain);
		driveTrain.setDefaultCommand(driveWithJoysticks);

		flywheel = new Flywheel();
		// outputFlywheelEncoder = new OutputFlywheelEncoder(flywheel);
		// flywheel.setDefaultCommand(outputFlywheelEncoder);
		flywheel.setDefaultCommand(new RunCommand(() -> flywheel.setSetpoint(1500), flywheel));
		runFlywheelFullSpeed = new RunFlywheelFullSpeed(flywheel);
		runShooterAtSetpoint = new RunShooterAtSetpoint(flywheel);

		tower = new Tower();
		kicker = new Kicker();

		runKicker = new RunKicker(kicker, tower);
		kicker.setDefaultCommand(runKicker);
		runTower = new RunTower(kicker, tower);
		tower.setDefaultCommand(runTower);

		accumulator = new Accumulator();
		runAccumulator = new RunAccumulator(accumulator, kicker, tower);
		accumulator.setDefaultCommand(runAccumulator);

		intake = new Intake();
		runIntakeRollers = new RunIntakeRollers(intake);
		intake.setDefaultCommand(runIntakeRollers);

		turret = new Turret();
		hood = new Hood();
		turretVision = new TurretVision();

		runKickerTest = new RunKickerTest(kicker);

		aimAndShoot = new AimAndShoot(flywheel, turret, hood, accumulator, tower, kicker, turretVision, driveTrain);
		shoot = new Shoot(flywheel, accumulator, tower, kicker);
		runKickerAndTower = new RunKickerandTower(kicker, tower);
		TimedShoot = new timedShoot(flywheel, accumulator, tower, kicker, Constants.AUTONOMOUS_SHOOT_TIMER);

		climb = new Climb();
		runClimb = new RunClimb(climb);
		climb.setDefaultCommand(runClimb);
		// autonDrive = new AutonDrive(driveTrain);

		simpleAuto = new SimpleAuton(flywheel, turret, hood, accumulator, tower, kicker, turretVision, driveTrain);

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
		OperatorInput.toggleIntakePistons.whenPressed(new InstantCommand(intake::togglePistons, intake));
		OperatorInput.toggleAimAndShoot.whenPressed(aimAndShoot);
		OperatorInput.toggleRunShooterAtSetpoint.whileHeld(runShooterAtSetpoint);
		OperatorInput.holdRunKickerTest.whileHeld(runKickerTest);
		OperatorInput.aimAndShootToggle.whileHeld(shoot);
		OperatorInput.runKickerAndTower.whileHeld(runKickerAndTower);
		// OperatorInput.toggleClimbTelescope.whenPressed(new InstantCommand(climb::toggleTelescopeLock, climb));
		
		OperatorInput.corunFlywheelFullButton.whileHeld(runFlywheelFullSpeed);
		OperatorInput.cotoggleIntakePistons.whenPressed(new InstantCommand(intake::togglePistons, intake));
		OperatorInput.cotoggleAimAndShoot.whenPressed(aimAndShoot);
		OperatorInput.cotoggleRunShooterAtSetpoint.whileHeld(runShooterAtSetpoint);
		OperatorInput.coholdRunKickerTest.whileHeld(runKickerTest);
		OperatorInput.coaimAndShootToggle.whileHeld(shoot);
		OperatorInput.corunKickerAndTower.whenPressed(new InstantCommand(climb::unlockTelescope, climb));
		OperatorInput.cotoggleClimbTelescope.whenPressed(new InstantCommand(climb::lockTelescope, climb));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return simpleAuto;
		// return null;

	}
}