// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Accumulator extends SubsystemBase {
	private CANSparkMax accumulatorMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushed);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.burnFlash();
		return sparkMax;
	}

	private CANSparkMax leftMotor = accumulatorMotor(Constants.ACCUMULATOR_MOTOR_PORTS[0], false);
	private CANSparkMax rightMotor = accumulatorMotor(Constants.ACCUMULATOR_MOTOR_PORTS[1], true);
	private MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

	/** Creates a new Accumulator. */
	public Accumulator() {
	}

	public void setMotors(Double speed) {
		motors.set(speed);
	}

	public void stopMotors() {
		motors.stopMotor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
