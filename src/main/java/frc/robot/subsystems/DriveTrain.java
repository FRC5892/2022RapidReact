// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
	private CANSparkMax leftMotor1 = driveMotor(1, true);
	private CANSparkMax leftMotor2 = driveMotor(2, true);
	private CANSparkMax leftMotor3 = driveMotor(3, true);
	private CANSparkMax rightMotor1 = driveMotor(4, false);
	private CANSparkMax rightMotor2 = driveMotor(5, false);
	private CANSparkMax rightMotor3 = driveMotor(6, false);

	private static final double kGearRatio = 4.444;
    private static final double kWheelRadiusInches = 120;

	// TODO evaluate connecting to spark maxes, make sparkmax sim work
	private Encoder leftEncoder = new Encoder(0, 1, true);
	private Encoder rightEncoder = new Encoder(2, 3, false);

	AHRS gyro = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06);

  PIDController leftPIDController = new PIDController(2.95, 0, 0);
  PIDController rightPIDController = new PIDController(2.95, 0, 0);

  Pose2d pose = new Pose2d();

	private MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2, leftMotor3);
	private MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2, rightMotor3);

	private DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

	// Create the simulation model of our drivetrain.
	// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
	private DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
			// fill in with characterization values once the chassis is done
			// LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
			DCMotor.getNEO(3), // 2 NEO motors on each side of the drivetrain.
			7.29, // 7.29:1 gearing reduction.
			7.5, // MOI of 7.5 kg m^2 (from CAD model).
			60.0, // The mass of the robot is 60 kg.
			Units.inchesToMeters(4), // The robot uses 3" radius wheels.
			0.7112, // The track width is 0.7112 meters.

			// The standard deviations for measurement noise:
			// x and y: 0.001 m
			// heading: 0.001 rad
			// l and r velocity: 0.1 m/s
			// l and r position: 0.005 m
			VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
	private EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
	private EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
	private final Field2d field = new Field2d();

	public CANSparkMax driveMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.setSmartCurrentLimit(60);
		sparkMax.burnFlash();
		return sparkMax;
	}

	public DriveTrain() {
		// TODO set distance per pulse and distance per rev
		// leftEncoder.setDistancePerPulse(distancePerRev/pulsesPerRev)
		// rightEncoder.setDistancePerPulse(distancePerRev/pulsesPerRev)
		SmartDashboard.putData(field);
		gyro.reset();
	}

	public void driveWithJoysticks(double xSpeed, double zRotation) {
		drive.arcadeDrive(xSpeed, zRotation);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Left Encoder", getLeftPosition());
		SmartDashboard.putNumber("Right Encoder", getRightPosition());
		pose = odometry.update(getHeading(), getLeftRate(), getRightRate());
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
		driveSim.setInputs(leftMotor1.get() * RobotController.getBatteryVoltage(),
				rightMotor1.get() * RobotController.getBatteryVoltage());
		driveSim.update(0.02);
		leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
		leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
		rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
		rightEncoderSim.setDistance(driveSim.getRightVelocityMetersPerSecond());
		field.setRobotPose(driveSim.getPose());

	}

	public Rotation2d getHeading() {
		return Rotation2d.fromDegrees(-gyro.getAngle());
	  }
	
	  public DifferentialDriveWheelSpeeds getSpeeds() {
		return new DifferentialDriveWheelSpeeds(
			getLeftPosition() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60,
			getRightPosition() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60
		);
	  }
	
	  public DifferentialDriveKinematics getKinematics() {
		return kinematics;
	  }
	
	  public Pose2d getPose() {
		return pose;
	  }
	
	  public SimpleMotorFeedforward getFeedforward() {
		return feedforward;
	  }

	  public void setOutputVolts(double leftVolts, double rightVolts) {
		leftMotors.set(leftVolts / 12);
		rightMotors.set(rightVolts / 12);
	  }
	
	  public void reset() {
		odometry.resetPosition(new Pose2d(), getHeading());
	  }

	  public PIDController getLeftPIDController() {
		return leftPIDController;
	  }
	
	  public PIDController getRightPIDController() {
		return rightPIDController;
	  }
	  

	// @Override
	public void stop() {
		drive.stopMotor();
	}

	public double getLeftPosition() {
		return leftEncoder.get() * Constants.ENCODER_CONVERSION_FACTOR;
	}

	public double getRightPosition() {
		return rightEncoder.get() * Constants.ENCODER_CONVERSION_FACTOR;
	}

	public double getLeftRate() {
		return leftEncoder.getRate() * Constants.ENCODER_CONVERSION_FACTOR;
	}

	public double getRightRate() {
		return rightEncoder.getRate() * Constants.ENCODER_CONVERSION_FACTOR;
	}

}
