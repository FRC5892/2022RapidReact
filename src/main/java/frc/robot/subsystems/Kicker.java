// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {
	private CANSparkMax kickerMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushed);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.burnFlash();
		return sparkMax;
	}

	private CANSparkMax kickerMotors = kickerMotor(Constants.KICKER_MOTOR_PORT, false);

	private DigitalInput kickerSensor = new DigitalInput(Constants.KICKER_SENSOR_PORT);

	/** Creates a new Kicker. */
	public Kicker() {
		// default
	}

	public void setMotors(double speed) {
		kickerMotors.set(speed);
	}

	public void stopMotors() {
		kickerMotors.stopMotor();
	}

	public boolean hasBall() {
		return kickerSensor.get();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
