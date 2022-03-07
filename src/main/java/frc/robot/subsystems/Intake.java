// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	private CANSparkMax intakeMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.burnFlash();
		return sparkMax;
	}

	private CANSparkMax leftMotor = intakeMotor(Constants.INTAKE_MOTOR_PORTS[0], false);
	private CANSparkMax rightMotor = intakeMotor(Constants.INTAKE_MOTOR_PORTS[1], true);
	private MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

	private DoubleSolenoid leftPrimarySolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			Constants.INTAKE_LEFT_PRIMARY_SOLENOID_PORTS[0], Constants.INTAKE_LEFT_PRIMARY_SOLENOID_PORTS[1]);
	private DoubleSolenoid rightPrimarySolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			Constants.INTAKE_RIGHT_PRIMARY_SOLENOID_PORTS[0], Constants.INTAKE_RIGHT_PRIMARY_SOLENOID_PORTS[1]);
	private DoubleSolenoid leftPositionSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			Constants.INTAKE_LEFT_POSITION_SOLENOID_PORTS[0], Constants.INTAKE_LEFT_POSITION_SOLENOID_PORTS[1]);
	private DoubleSolenoid rightPositionSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			Constants.INTAKE_RIGHT_POSITION_SOLENOID_PORTS[0], Constants.INTAKE_RIGHT_POSITION_SOLENOID_PORTS[1]);

	/** Creates a new Intake. */
	public Intake() {
		leftPrimarySolenoid.set(Value.kReverse);
		rightPrimarySolenoid.set(Value.kReverse);
		leftPositionSolenoid.set(Value.kReverse);
		rightPositionSolenoid.set(Value.kReverse);
	}

	public void setMotors(double speed) {
		if (leftPositionSolenoid.get() == Value.kForward) {
			motors.set(-speed);
		}
		else {
			motors.set(speed);
		}
	}

	public void togglePrimarySolenoids() {
		leftPrimarySolenoid.toggle();
		rightPrimarySolenoid.toggle();
	}

	public void togglePositionSolenoids() {
		leftPositionSolenoid.toggle();
		rightPositionSolenoid.toggle();
	}

	public DoubleSolenoid.Value primarySolenoidPosition() {
		return leftPrimarySolenoid.get();
	}

	public void setPositionSolenoids(DoubleSolenoid.Value value) {
		leftPrimarySolenoid.set(value);
		rightPrimarySolenoid.set(value);
	}

	public void stopMotors() {
		motors.stopMotor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
