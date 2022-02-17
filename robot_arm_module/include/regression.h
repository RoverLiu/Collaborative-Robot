/**
 * @file regression.h
 * @author Rover
 * @brief Handle the linear regression with given data or give coefficients
 * @version 0.1
 * @date 2022-02-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __REGRESSION_H
#define __REGRESSION_H

// C++ program to implement
// the above approach
#include <iostream>
#include <stdio.h>
#include <vector>
// using namespace std;
class regression {
	private:
		// Dynamic arrary which is going
		// to contain all (i-th x)
		std::vector<float> x;

		// Dynamic arrary which is going
		// to contain all (i-th y)
		std::vector<float> y;

		// Store the coefficient/slope in
		// the best fitting line
		float coeff;

		// Store the constant term in
		// the best fitting line
		float constTerm;

		// Contains sum of product of
		// all (i-th x) and (i-th y)
		float sum_xy;

		// Contains sum of all (i-th x)
		float sum_x;

		// Contains sum of all (i-th y)
		float sum_y;

		// Contains sum of square of
		// all (i-th x)
		float sum_x_square;

		// Contains sum of square of
		// all (i-th y)
		float sum_y_square;

	public:
		// Constructor to provide the default
		// values to all the terms in the
		// object of class regression
		regression(std::vector<float> x, std::vector<float> y);
		
		// another constructor with default values
		regression(float coeff, float constTerm);

		// Function that calculate the coefficient/
		// slope of the best fitting line
		void calculateCoefficient();

		// Member function that will calculate
		// the constant term of the best
		// fitting line
		void calculateConstantTerm();

		// Function that return the number
		// of entries (xi, yi) in the data set
		int sizeOfData();

		// Function that return the coeffecient/
		// slope of the best fitting line
		float getCoefficient();

		// Function that return the constant
		// term of the best fitting line
		float getConstant();

		// Function that print the best
		// fitting line
		void PrintBestFittingLine();

		// Function to take input from the dataset
		void calculateBasics();

		// Function to show the data set
		void showData();

		// Function to predict the value
		// correspondng to some input
		float predict(float x);

		// Function to predict the value
		// correspondng to some input
		float revert(float y);

		// Function that returns overall
		// sum of square of errors
		float errorSquare();

		// Functions that return the error
		// i.e the difference between the
		// actual value and value predicted
		// by our model
		float errorIn(float num);
};

// // Driver code
// int main()
// {
// 	freopen("input.txt", "r",
// 			stdin);
// 	regression reg;

// 	// Number of pairs of (xi, yi)
// 	// in the dataset
// 	int n;
// 	cin >> n;

// 	// Calling function takeInput to
// 	// take input of n pairs
// 	reg.takeInput(n);

// 	// Printing the best fitting line
// 	reg.PrintBestFittingLine();
// 	cout << "Predicted value at 2060 = "
// 		<< reg.predict(2060) << endl;
// 	cout << "The errorSquared = "
// 		<< reg.errorSquare() << endl;
// 	cout << "Error in 2050 = "
// 		<< reg.errorIn(2050) << endl;
// }



#endif