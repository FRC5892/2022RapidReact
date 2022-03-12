// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private CANSparkMax climbMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		// sparkMax.setSmartCurrentLimit(60);
		sparkMax.burnFlash();
		return sparkMax;
	}

  private CANSparkMax leftMotor = climbMotor(Constants.CLIMB_MOTOR_PORTS[0], false);
  private CANSparkMax rightMotor = climbMotor(Constants.CLIMB_MOTOR_PORTS[1], true);
  /** Creates a new Climb. */
  public Climb() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
