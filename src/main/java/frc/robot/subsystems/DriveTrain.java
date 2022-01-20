// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
	private CANSparkMax leftMotor1;
	private CANSparkMax leftMotor2;
	private CANSparkMax leftMotor3;
	private CANSparkMax rightMotor1;
	private CANSparkMax rightMotor2;
	private CANSparkMax rightMotor3;
	private MotorControllerGroup leftMotors;
	private MotorControllerGroup rightMotors;
	private DifferentialDrive drive;

	public CANSparkMax driveMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.burnFlash();
		return sparkMax;
	}

	public DriveTrain() {
		leftMotor1 = driveMotor(1, true);
		leftMotor2 = driveMotor(2, true);
		leftMotor3 = driveMotor(3, true);
		rightMotor1 = driveMotor(4, false);
		rightMotor2 = driveMotor(5, false);
		rightMotor3 = driveMotor(6, false);

		leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2, leftMotor3);
		rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2, rightMotor3);

		drive = new DifferentialDrive(leftMotors, rightMotors);

	}

	public void driveWithJoysticks(double xSpeed, double zRotation) {
		drive.arcadeDrive(xSpeed, zRotation);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void stop() {
		drive.stopMotor();
	}

	public double getLeftPosition() {
		double position = leftMotor1.getEncoder().getPosition();
		return position;
	}

	public double getRightPosition() {
		double position = rightMotor1.getEncoder().getPosition();
		return position;
	}

	public void resetEncoders() {
		leftMotor1.getEncoder().setPosition(0);
		rightMotor1.getEncoder().setPosition(0);
		System.out.println("Resetting Encoders");
	}

	public void driveForward(double speed) {
		drive.tankDrive(speed, speed);
	}
}
