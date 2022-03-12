// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

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

public class Hood extends PIDSubsystem {
	private AnalogInput potentiometer = new AnalogInput(Constants.HOOD_POTENTIOMETER_PORT);

	public CANSparkMax turretMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		// sparkMax.restoreFactoryDefaults();
		// this motor needs to be programmed with rev hardware client so that the limit switches work properly
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.setSmartCurrentLimit(15);
		sparkMax.burnFlash();
		return sparkMax;
	}

	private CANSparkMax motor = turretMotor(Constants.HOOD_MOTOR_ID, false);
	// weird rev API issue where limit
	private SparkMaxLimitSwitch topLimit = motor.getForwardLimitSwitch(Type.kNormallyClosed);
	private SparkMaxLimitSwitch bottomLimit = motor.getReverseLimitSwitch(Type.kNormallyClosed);

	/** Creates a new Hood. */
	public Hood() {
		super(
				// The PIDController used by the subsystem
				new PIDController(Constants.HOOD_PID[0], Constants.HOOD_PID[1], Constants.HOOD_PID[2]));
		topLimit.enableLimitSwitch(false);
		bottomLimit.enableLimitSwitch(false);
		SmartDashboard.putNumber("Hood P", Constants.HOOD_PID[0]);
		SmartDashboard.putNumber("Hood I", Constants.HOOD_PID[1]);
		SmartDashboard.putNumber("Hood D", Constants.HOOD_PID[2]);
		SmartDashboard.putNumber("Hood Setpoint", 0);
		this.enable();
	}

	public void stop() {
		this.disable();
	}

	public double getAngle() {
		// TODO set conversion factor
		return potentiometer.getVoltage() * (-4.87912) + 50.7509;
		// return motor.getEncoder().getPosition() * 1 + 1;
	}

	public boolean atSetpoint() {
		return this.m_controller.atSetpoint();
	}

	@Override
	public void useOutput(double output, double setpoint) {
		// Use the output here
		// if (!topLimit.isPressed() && output > 0) {
		// motor.set(output);
		// }
		// else if (bottomLimit.isPressed() && output < 0) {
		// motor.set(output);
		// }
		// else if (!output == 0) {
		// motor.set(output);
		// System.out.println("Running");
		// }
		// inverted because revlib won't do its job
		if (Math.abs(output) >= 0.1) {
			motor.set(.1);
		}
		else {
			motor.set(output);
		}
		SmartDashboard.putBoolean("Hood Top Limit", topLimit.isPressed());
		SmartDashboard.putBoolean("Hood Bottom Limit", bottomLimit.isPressed());
		SmartDashboard.putNumber("Hood Pot Voltage", potentiometer.getVoltage());
		SmartDashboard.putNumber("Hood Angle", getAngle());
		SmartDashboard.putNumber("Hood Set Setpoint", setpoint);
		this.m_controller.setP(SmartDashboard.getNumber("Hood P", 0));
		this.m_controller.setI(SmartDashboard.getNumber("Hood I", 0));
		this.m_controller.setD(SmartDashboard.getNumber("Hood D", 0));
		this.setSetpoint(SmartDashboard.getNumber("Hood Setpoint", 0));
	}

	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		SmartDashboard.putNumber("Hood Encoder", getAngle());
		return getAngle();
	}
}
