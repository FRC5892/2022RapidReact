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
	public static JoystickButton toggleIntakePistons = new JoystickButton(driverJoystick,
			XboxController.Button.kLeftBumper.value);
	public static JoystickButton toggleAimAndShoot = new JoystickButton(driverJoystick, XboxController.Button.kX.value);
	public static JoystickButton toggleRunShooterAtSetpoint = new JoystickButton(driverJoystick,
			XboxController.Button.kStart.value);
	public static JoystickButton holdRunKickerTest = new JoystickButton(driverJoystick,
			XboxController.Button.kBack.value);
	public static JoystickButton runKickerAndTower = new JoystickButton(driverJoystick, XboxController.Button.kA.value);
	public static JoystickButton aimAndShootToggle = new JoystickButton(driverJoystick,
			XboxController.Button.kRightBumper.value);
	// public static JoystickButton toggleClimbTelescope = new JoystickButton(driverJoystick, XboxController.Button.kB.value);


	public static XboxController codriverJoystick = new XboxController(1);
	public static JoystickButton corunFlywheelFullButton = new JoystickButton(codriverJoystick,
	XboxController.Button.kY.value);
	public static JoystickButton cotoggleIntakePistons = new JoystickButton(codriverJoystick,
		XboxController.Button.kLeftBumper.value);
	public static JoystickButton cotoggleAimAndShoot = new JoystickButton(codriverJoystick, XboxController.Button.kX.value);
	public static JoystickButton cotoggleRunShooterAtSetpoint = new JoystickButton(driverJoystick,
		XboxController.Button.kStart.value);
	public static JoystickButton coholdRunKickerTest = new JoystickButton(codriverJoystick,
		XboxController.Button.kBack.value);
	public static JoystickButton corunKickerAndTower = new JoystickButton(codriverJoystick, XboxController.Button.kA.value);
	public static JoystickButton coaimAndShootToggle = new JoystickButton(codriverJoystick,
		XboxController.Button.kRightBumper.value);
	public static JoystickButton cotoggleClimbTelescope = new JoystickButton(codriverJoystick, XboxController.Button.kB.value);
}
