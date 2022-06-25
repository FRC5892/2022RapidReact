// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.AimDriveTrain;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.RunClimb;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.serializer.Intake;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.TurretVision;
import frc.robot.commands.autonomous.AutonSetup1;
import frc.robot.commands.serializing.RunIntakeRollers;
import frc.robot.commands.serializing.RunKicker;
import frc.robot.commands.serializing.RunKickerManual;
import frc.robot.commands.serializing.ReverseKickerAndTower;
import frc.robot.commands.serializing.RunTower;
import frc.robot.commands.shooting.AimAndShoot;
import frc.robot.commands.shooting.FlywheelHoodTuningShoot;
import frc.robot.commands.shooting.LaunchPadShot;
import frc.robot.commands.shooting.PrespoolFlywheel;
import frc.robot.commands.shooting.RunShooterAtSetpoint;
import frc.robot.commands.shooting.Shoot;
import frc.robot.commands.shooting.TapeShot;

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

	private Intake intake;

	private RunIntakeRollers runIntakeRollers;
	private Tower tower;
	private Kicker kicker;
	private Hood hood;
	private Turret turret;

	private AimAndShoot aimAndShoot;
	private TurretVision turretVision;

	private RunShooterAtSetpoint runShooterAtSetpoint;
	private ReverseKickerAndTower reverseKickerAndTower;
	private RunKickerManual runKickerManual;

	private AutonSetup1 simpleAuto;

	private RunKicker runKicker;

	private RunTower runTower;

	private Compressor compressor;

	private Climb climb;

	private RunClimb runClimb;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */

	private PrespoolFlywheel prespoolFlywheel;

	private FlywheelHoodTuningShoot flywheelHoodTuningShoot;

	private LaunchPadShot launchpadShot;

	private TapeShot tapeshot;

	public RobotContainer() {

		compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
		compressor.enableDigital();

		driveTrain = new DriveTrain();
		driveWithJoysticks = new DriveWithJoysticks(driveTrain);
		driveTrain.setDefaultCommand(driveWithJoysticks);

		flywheel = new Flywheel();
		// outputFlywheelEncoder = new OutputFlywheelEncoder(flywheel);
		// flywheel.setDefaultCommand(outputFlywheelEncoder);
		prespoolFlywheel = new PrespoolFlywheel(flywheel);
		flywheel.setDefaultCommand(prespoolFlywheel);
		runShooterAtSetpoint = new RunShooterAtSetpoint(flywheel);

		tower = new Tower();
		kicker = new Kicker();

		runKicker = new RunKicker(kicker, tower);
		kicker.setDefaultCommand(runKicker);
		runTower = new RunTower(kicker, tower);
		tower.setDefaultCommand(runTower);

		

		intake = new Intake();
		runIntakeRollers = new RunIntakeRollers(intake, tower, kicker);
		intake.setDefaultCommand(runIntakeRollers);

		turret = new Turret();
		hood = new Hood();
		turretVision = new TurretVision();

		runKickerManual = new RunKickerManual(kicker);

		launchpadShot = new LaunchPadShot(flywheel, turret, hood, tower, kicker, turretVision, driveTrain);
		tapeshot = new TapeShot(flywheel, turret, hood, tower, kicker, turretVision, driveTrain);

		aimAndShoot = new AimAndShoot(flywheel, turret, hood, tower, kicker, turretVision, driveTrain);
		new Shoot(flywheel, tower, kicker, hood, Constants.FLYWHEEL_SHOOTING_SPEED,
				Constants.FLYWHEEL_SHOOTING_ANGLE);
		new Shoot(flywheel, tower, kicker, hood, Constants.FLYWHEEL_LONG_SHOOTING_SPEED,
				Constants.FLYWHEEL_LONG_SHOOTING_ANGLE);
		reverseKickerAndTower = new ReverseKickerAndTower(kicker, tower);
		flywheelHoodTuningShoot = new FlywheelHoodTuningShoot(flywheel, tower, kicker, hood);
		new AimDriveTrain(driveTrain, turretVision);

		climb = new Climb();
		runClimb = new RunClimb(climb);
		climb.setDefaultCommand(runClimb);
		// autonDrive = new AutonDrive(driveTrain);
		SmartDashboard.putData(CommandScheduler.getInstance());

		simpleAuto = new AutonSetup1(flywheel, turret, hood, tower, kicker, turretVision, driveTrain, intake);

		// Configure the button bindingsz
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by instantiating a
	 * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
	 * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		OperatorInput.toggleIntakePistons.whenPressed(new InstantCommand(intake::togglePistons, intake));
		OperatorInput.holdRunKickerManual.whileHeld(runKickerManual);
		OperatorInput.shootFromSafe.whileHeld(launchpadShot);
		OperatorInput.holdReverseKickerAndTower.whileHeld(reverseKickerAndTower);
		OperatorInput.holdFlywheelTuning.whileHeld(flywheelHoodTuningShoot);
		OperatorInput.shootFromTape.whileHeld(tapeshot);
		OperatorInput.aimandshootcomplex.whenHeld(aimAndShoot);
		//OperatorInput.toggleAimAndShoot.whenPressed(aimAndShoot);
		//OperatorInput.toggleRunShooterAtSetpoint.whileHeld(shoot);
		// OperatorInput.aimAndShootToggle.whileHeld(shoot);
		//OperatorInput.holdLongShot.whileHeld(longShot);
		// OperatorInput.aimAndShootToggle.whileHeld(shoot);
		//OperatorInput.holdLongShot.whileHeld(longShot);
		//OperatorInput.holdPointDriveTrain.whileHeld(aimDriveTrain);

		OperatorInput.cotoggleIntakePistons.whenPressed(new InstantCommand(intake::togglePistons, intake));
		OperatorInput.cotoggleAimAndShoot.whenPressed(aimAndShoot);
		OperatorInput.cotoggleRunShooterAtSetpoint.whileHeld(runShooterAtSetpoint);
		OperatorInput.coholdRunKickerManual.whileHeld(runKickerManual);
		OperatorInput.cotoggleClimbPistons.whenPressed(new InstantCommand(climb::togglePistons, climb));
		//OperatorInput.corunKickerAndTower.whenPressed(new InstantCommand(climb::unlockTelescope, climb));
		//OperatorInput.cotoggleClimbTelescope.whenPressed(new InstantCommand(climb::lockTelescope, climb));
		
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);
		TrajectoryConfig config =
		new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
								Constants.kMaxAccelerationMetersPerSecondSquared)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(Constants.kDriveKinematics)
			// Apply the voltage constraint
			.addConstraint(autoVoltageConstraint);
		
			Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
		// Start at the origin facing the +X direction
		new Pose2d(0, 0, new Rotation2d(0)),
		// Pass through these two interior waypoints, making an 's' curve path
		List.of(
			new Translation2d(1, 1),
			new Translation2d(2, -1)
		),
		// End 3 meters straight ahead of where we started, facing forward
		new Pose2d(3, 0, new Rotation2d(0)),
		// Pass config
		config
    );
	
	RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        driveTrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
		Constants.kvVoltSecondsPerMeter,
		Constants.kaVoltSecondsSquaredPerMeter),
		Constants.kDriveKinematics,
		driveTrain::getWheelSpeeds,
		new PIDController(Constants.kPDriveVel, 0, 0),
		new PIDController(Constants.kPDriveVel, 0, 0),
		// RamseteCommand passes volts to the callback
		driveTrain::tankDriveVolts,
		driveTrain
		);
		
		driveTrain.resetOdometry(exampleTrajectory.getInitialPose());
		return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
								   
	}
}
