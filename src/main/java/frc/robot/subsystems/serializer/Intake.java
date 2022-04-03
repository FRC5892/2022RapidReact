// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.serializer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	private CANSparkMax motor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.burnFlash();
		return sparkMax;
	}

	private CANSparkMax motor;
	private DoubleSolenoid pistons;

	/** Creates a new Intake. */
	public Intake() {
		motor = motor(Constants.INTAKE_MOTOR_PORT, false);
		pistons = new DoubleSolenoid(
			PneumaticsModuleType.CTREPCM,
			Constants.INTAKE_PISTON_SOLENOID_PORTS[0], 
			Constants.INTAKE_PISTON_SOLENOID_PORTS[1]
		);
		pistons.set(Value.kReverse);
	}

	public void setMotors(double speed) {
		motor.set(speed);
	}

	public void togglePistons() {
		pistons.toggle();
	}

	public void setPistons(DoubleSolenoid.Value value) {
		pistons.set(value);
	}

	public void stopMotors() {
		motor.stopMotor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
