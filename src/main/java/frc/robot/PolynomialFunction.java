// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * This function takes an array of coefficients (ordered from smallest to largest degree) and an x value and returns the
 * result of the polynomial with the x value plugged in to it
 *
 * @author Micah Guttman
 * @param x
 *            This is the value that will be plugged in to the polynomial
 * @param coefficients
 *            This is the array of coefficients that will be used to construct the polynomial
 */
public class PolynomialFunction {
	public static double polynomialFunction(double x, double[] coefficients) {
		double value = 0;
		int i = 0;
		for (double coefficient : coefficients) {
			if (i > 0) {
				value = value + coefficient * (Math.pow(x, i));
			}
			else {
				value = coefficient;
			}
			i++;
		}
		return value;
	}
}
