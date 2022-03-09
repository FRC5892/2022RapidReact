// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class OperatorInput {
	public static XboxController driverJoystick = new XboxController(0);
	public static JoystickButton runFlywheelFullButton = new JoystickButton(driverJoystick,
			XboxController.Button.kY.value);
	public static JoystickButton toggleIntake = new JoystickButton(driverJoystick,
			XboxController.Button.kLeftBumper.value);
	public static JoystickButton toggleIntakePosition = new JoystickButton(driverJoystick,
			XboxController.Button.kRightBumper.value);
	public static JoystickButton toggleAimAndShoot = new JoystickButton(driverJoystick, XboxController.Button.kX.value);
	public static JoystickButton toggleRunShooterAtSetpoint = new JoystickButton(driverJoystick,
			XboxController.Button.kStart.value);
	public static JoystickButton holdRunKickerTest = new JoystickButton(driverJoystick,
			XboxController.Button.kBack.value);

}
