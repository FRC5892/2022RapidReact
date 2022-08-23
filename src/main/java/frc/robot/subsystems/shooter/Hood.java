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

public class Hood extends PIDSubsystem {
	private AnalogInput potentiometer = new AnalogInput(Constants.HOOD_POTENTIOMETER_PORT);

	public CANSparkMax hoodMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		// sparkMax.restoreFactoryDefaults();
		// this motor needs to be programmed with rev hardware client so that the limit switches work properly
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.setSmartCurrentLimit(15);
		
		return sparkMax;
	}

	private CANSparkMax motor = hoodMotor(Constants.HOOD_MOTOR_ID, true);
	// weird rev API issue where limit
	private SparkMaxLimitSwitch topLimit = motor.getForwardLimitSwitch(Type.kNormallyClosed);
	private SparkMaxLimitSwitch bottomLimit = motor.getReverseLimitSwitch(Type.kNormallyClosed);

	/** Creates a new Hood. */
	public Hood() {
		super(
				// The PIDController used by the subsystem
				new PIDController(Constants.HOOD_PID[0], Constants.HOOD_PID[1], Constants.HOOD_PID[2]));
		topLimit.enableLimitSwitch(true);
		bottomLimit.enableLimitSwitch(true);
		SmartDashboard.putNumber("Hood P", Constants.HOOD_PID[0]);
		SmartDashboard.putNumber("Hood I", Constants.HOOD_PID[1]);
		SmartDashboard.putNumber("Hood D", Constants.HOOD_PID[2]);
		SmartDashboard.putNumber("Hood Setpoint", 0);
		this.enable();
		this.m_controller.setTolerance(.5);
	}

	public void stopMotors() {
		this.disable();
		motor.stopMotor();
	}

	public double getAngle() {
		return potentiometer.getVoltage() * (-29.4784) + 85.7506+4.3+1.3;
	}

	public boolean atSetpoint() {
		return this.m_controller.atSetpoint();
	}

	@Override
	public void useOutput(double output, double setpoint) {
		if (output > 0.15) {
			motor.set(0.15);
		}
		else if (output < -0.15) {
			motor.set(-0.15);
		}
		else {
			motor.set(output);
		}

		// PID Tuning Stuff
		SmartDashboard.putBoolean("Hood Top Limit", topLimit.isPressed());
		SmartDashboard.putBoolean("Hood Bottom Limit", bottomLimit.isPressed());
		SmartDashboard.putNumber("Hood Pot Voltage", potentiometer.getVoltage());
		SmartDashboard.putNumber("Hood Angle", getAngle());
		SmartDashboard.putNumber("Hood Set Setpoint", setpoint);
		SmartDashboard.putBoolean("Hood At Setpoint", this.atSetpoint());
		// this.m_controller.setP(SmartDashboard.getNumber("Hood P", 0));
		// this.m_controller.setI(SmartDashboard.getNumber("Hood I", 0));
		// this.m_controller.setD(SmartDashboard.getNumber("Hood D", 0));
		// this.setSetpoint(SmartDashboard.getNumber("Hood Setpoint", 0));
		SmartDashboard.putNumber("Hood Motor Commanded", motor.get());
	}

	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		SmartDashboard.putNumber("Hood Encoder", getAngle());
		return getAngle();
	}
}
