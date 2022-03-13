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

	private CANSparkMax motor = intakeMotor(Constants.INTAKE_MOTOR_PORT, false);

	private DoubleSolenoid primarySolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			Constants.INTAKE_PRIMARY_SOLENOID_PORTS[0], Constants.INTAKE_PRIMARY_SOLENOID_PORTS[1]);
	private DoubleSolenoid positionSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			Constants.INTAKE_POSITION_SOLENOID_PORTS[0], Constants.INTAKE_POSITION_SOLENOID_PORTS[1]);

	/** Creates a new Intake. */
	public Intake() {
		primarySolenoid.set(Value.kReverse);
		positionSolenoid.set(Value.kReverse);
	}

	public void setMotors(double speed) {
		// logic for second position actuation
		// if (positionSolenoid.get() == Value.kForward) {
		// 	motor.set(-speed);
		// }
		// else {
		// 	motor.set(speed);
		// }
		motor.set(speed);
	}

	public void togglePrimarySolenoids() {
		primarySolenoid.toggle();
	}

	public void togglePositionSolenoids() {
		positionSolenoid.toggle();
	}

	public DoubleSolenoid.Value primarySolenoidPosition() {
		return primarySolenoid.get();
	}

	public void setPositionSolenoids(DoubleSolenoid.Value value) {
		primarySolenoid.set(value);
	}

	public void stopMotors() {
		motor.stopMotor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
