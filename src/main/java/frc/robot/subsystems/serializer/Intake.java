// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj.Timer;

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
		sparkMax.setSmartCurrentLimit(20);
		
		return sparkMax;
	}

	private CANSparkMax motor = motor(Constants.INTAKE_MOTOR_PORT, false);
	private DoubleSolenoid pistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
			Constants.INTAKE_PISTON_SOLENOID_PORTS[0], Constants.INTAKE_PISTON_SOLENOID_PORTS[1]);
	private Timer timer;
	/** Creates a new Intake. */
	public Intake() {
		pistons.set(Value.kReverse);
		timer = new Timer();
		timer.reset();
	}

	public void setMotors(double speed) {
		motor.set(speed);
	}

	public double getMotors(){
		return motor.get();
	}

	public void togglePistons() {
		pistons.toggle();
	}

	public void openPistons() {
			pistons.set(Value.kForward);
		
	}

	public void closePistons() {
		pistons.set(Value.kReverse);
		}


	public void setPistons(DoubleSolenoid.Value value) {
		pistons.set(value);
	}

	public Value returnPistons(){
		return pistons.get();
	}

	public void stopMotors() {
		motor.stopMotor();
	}

	public void runMotorFoward(){
		motor.set(-1);
	}

	

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
