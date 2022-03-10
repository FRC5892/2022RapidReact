// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Flywheel extends PIDSubsystem {
	private CANSparkMax leftMotor = shooterMotor(Constants.FLYWHEEL_MOTOR_IDS[0], false);
	private CANSparkMax rightMotor = shooterMotor(Constants.FLYWHEEL_MOTOR_IDS[1], true);
	private MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

	private Encoder encoder = new Encoder(Constants.FLYWHEEL_ENCODER_PORTS[0], Constants.FLYWHEEL_ENCODER_PORTS[1]);

	private RelativeEncoder neoEncoder = leftMotor.getEncoder(); 

	public CANSparkMax shooterMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kCoast);
		sparkMax.burnFlash();
		return sparkMax;
	}

	/** Creates a new Shooter. */
	public Flywheel() {
		super(
				// The PIDController used by the subsystem
				new PIDController(Constants.FLYWHEEL_PID_CONSTANTS[0], Constants.FLYWHEEL_PID_CONSTANTS[1], Constants.FLYWHEEL_PID_CONSTANTS[2]));
		SmartDashboard.putNumber("Flywheel P", this.m_controller.getP());
		SmartDashboard.putNumber("Flywheel I", this.m_controller.getI());
		SmartDashboard.putNumber("Flywheel D", this.m_controller.getD());
		SmartDashboard.putNumber("Flywheel Setpoint", this.m_controller.getSetpoint());


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

	@Override
	public void useOutput(double output, double setpoint) {
		// Use the output here
		motors.set(output);
		SmartDashboard.putNumber("Flywheel Velocity", getVelocity());
		this.m_controller.setP(SmartDashboard.getNumber("Flywheel P", 0));
		this.m_controller.setI(SmartDashboard.getNumber("Flywheel I", 0));
		this.m_controller.setD(SmartDashboard.getNumber("Flywheel D", 0));
		this.setSetpoint(SmartDashboard.getNumber("Flywheel Setpoint", 0));

	}

	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		return getVelocity();
	}
}
