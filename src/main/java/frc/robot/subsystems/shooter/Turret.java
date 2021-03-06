// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Turret extends PIDSubsystem {
	private AnalogInput potentiometer = new AnalogInput(Constants.TURRET_POTENTIOMETER);

	public CANSparkMax turretMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		
		return sparkMax;
	}

	private CANSparkMax motor = turretMotor(Constants.TURRET_MOTOR_ID, false);
	private SparkMaxLimitSwitch leftLimit = motor.getForwardLimitSwitch(Type.kNormallyClosed);
	private SparkMaxLimitSwitch rightLimit = motor.getReverseLimitSwitch(Type.kNormallyClosed);

	/** Creates a new Hood. */
	public Turret() {
		super(
				// The PIDController used by the subsystem
				new PIDController(0, 0, 0));
		leftLimit.enableLimitSwitch(true);
		rightLimit.enableLimitSwitch(true);
		SmartDashboard.putData("Turret PID", this.m_controller);
	}

	public void stop() {
		motor.stopMotor();
		this.disable();
	}

	public void setMotor(double speed) {
		motor.set(speed);
	}

	public double getAngle() {
		return potentiometer.getVoltage() * 1 + 1;
	}

	public boolean atLimit() {
		return leftLimit.isPressed() || rightLimit.isPressed();
	}

	public boolean atLeftLimit() {
		return leftLimit.isPressed();
	}

	public boolean atRightLimit() {
		return rightLimit.isPressed();
	}

	public boolean atSetpoint() {
		return this.m_controller.atSetpoint();
	}

	@Override
	public void useOutput(double output, double setpoint) {
		// Use the output here
		motor.set(output);
		SmartDashboard.putBoolean("Turret Top Limit", leftLimit.isPressed());
		SmartDashboard.putBoolean("Turret Bottom Limit", rightLimit.isPressed());
	}

	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		return getAngle();
	}
}
