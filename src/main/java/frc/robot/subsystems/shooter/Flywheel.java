// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Flywheel extends PIDSubsystem {

	// private Encoder encoder = new Encoder(Constants.FLYWHEEL_ENCODER_PORTS[0], Constants.FLYWHEEL_ENCODER_PORTS[1]);

	public CANSparkMax shooterMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kCoast);
		sparkMax.burnFlash();
		return sparkMax;
	}

	private CANSparkMax leftMotor = shooterMotor(Constants.FLYWHEEL_MOTOR_IDS[0], false);
	private CANSparkMax rightMotor = shooterMotor(Constants.FLYWHEEL_MOTOR_IDS[1], true);
	private MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);
	private RelativeEncoder neoEncoder = leftMotor.getEncoder();

	/** Creates a new Shooter. */
	public Flywheel() {
		super(
				// The PIDController used by the subsystem
				new PIDController(Constants.FLYWHEEL_PID_CONSTANTS[0], Constants.FLYWHEEL_PID_CONSTANTS[1],
						Constants.FLYWHEEL_PID_CONSTANTS[2]));
		this.disable();
		this.m_controller.setTolerance(100);
	}

	public void setMotors(double speed) {
		motors.set(speed);
	}

	public void stop() {
		this.disable();
	}

	public double getVelocity() {
		return neoEncoder.getVelocity() * Constants.FLYWHEEL_ENCODER_CONVERSION_FACTOR;
	}

	public double getPosition() {
		return neoEncoder.getPosition() * Constants.FLYWHEEL_ENCODER_CONVERSION_FACTOR;
	}

	public boolean atSetpoint() {
		return this.m_controller.atSetpoint();
	}

	@Override
	public void useOutput(double output, double setpoint) {
		// Use the output here
		motors.set(output);
		SmartDashboard.putNumber("Flywheel Velocity", getVelocity());
		SmartDashboard.putBoolean("Flywheel At Setpoint", atSetpoint());

	}

	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		return getVelocity();
	}
}
