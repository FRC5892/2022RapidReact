// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package PolynomialFunction;

/** Add your docs here. */
public class PolynomialFunction {
	public double polynomailFunction(double x, double[] coefficients) {
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
