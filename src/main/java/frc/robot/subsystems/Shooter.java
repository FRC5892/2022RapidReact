// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Shooter extends PIDSubsystem {
	private CANSparkMax leftMotor = shooterMotor(7, false);
	private CANSparkMax rightMotor = shooterMotor(8, true);
	private MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

	private Encoder encoder = new Encoder(Constants.SHOOTER_ENCODER_PORTS[1], Constants.SHOOTER_ENCODER_PORTS[2]);

	public CANSparkMax shooterMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kCoast);
		sparkMax.burnFlash();
		return sparkMax;
	}

	/** Creates a new Shooter. */
	public Shooter() {
		super(
				// The PIDController used by the subsystem
				new PIDController(0, 0, 0));

	}

	public void setMotors(double speed) {
		motors.set(speed);
	}

	public void stop() {
		motors.stopMotor();
	}

	public double getVelocity() {
		return encoder.getRate() * Constants.SHOOTER_ENCODER_CONVERSION_FACTOR;
	}

	@Override
	public void useOutput(double output, double setpoint) {
		// Use the output here
		motors.set(output);
	}

	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		return getVelocity();
	}
}
