// C++ program to implement
// the above approach
#include <iostream>
#include <stdio.h>
#include <vector>
// using namespace std;
#include "regression.h"


// Constructor to provide the default
// values to all the terms in the
// object of class regression
regression::regression(std::vector<float> x, std::vector<float> y) :
x(x), y(y)
{
    coeff = 0;
    constTerm = 0;
    sum_y = 0;
    sum_y_square = 0;
    sum_x_square = 0;
    sum_x = 0;
    sum_xy = 0;

    calculateBasics();
}

// another constructor with default values
regression::regression(float coeff, float constTerm) :
coeff(coeff), constTerm(constTerm)
{}

// Function that calculate the coefficient/
// slope of the best fitting line
void regression::calculateCoefficient()
{
    float N = x.size();
    float numerator
        = (N * sum_xy - sum_x * sum_y);
    float denominator
        = (N * sum_x_square - sum_x * sum_x);
    coeff = numerator / denominator;
}

// Member function that will calculate
// the constant term of the best
// fitting line
void regression::calculateConstantTerm()
{
    float N = x.size();
    float numerator
        = (sum_y * sum_x_square - sum_x * sum_xy);
    float denominator
        = (N * sum_x_square - sum_x * sum_x);
    constTerm = numerator / denominator;
}

// Function that return the number
// of entries (xi, yi) in the data set
int regression::sizeOfData()
{
    return x.size();
}

// Function that return the coeffecient/
// slope of the best fitting line
float regression::getCoefficient()
{
    if (coeff == 0)
        calculateCoefficient();
    return coeff;
}

// Function that return the constant
// term of the best fitting line
float regression::getConstant()
{
    if (constTerm == 0)
        calculateConstantTerm();
    return constTerm;
}

// Function that print the best
// fitting line
void regression::PrintBestFittingLine()
{
    if (coeff == 0 && constTerm == 0) {
        calculateCoefficient();
        calculateConstantTerm();
    }
    printf("The best fitting line is y = %.5fx+%f\n", coeff, constTerm);
    // std::cout << "The best fitting line is y = "
    //     << coeff << "x + " << constTerm << std::endl;
}

// Function to take input from the dataset
void regression::calculateBasics()
{
    for (int i = 0; i < sizeOfData(); i++) {
        // In a csv file all the values of
        // xi and yi are separated by commas
        char comma;
        float xi = x.at(i);
        float yi = y.at(i);
        // std::cin >> xi >> comma >> yi;
        sum_xy += xi * yi;
        sum_x += xi;
        sum_y += yi;
        sum_x_square += xi * xi;
        sum_y_square += yi * yi;
        // x.push_back(xi);
        // y.push_back(yi);
    }
}

// Function to show the data set
void regression::showData()
{
    for (int i = 0; i < 62; i++) {
        printf("_");
    }
    printf("\n\n");
    printf("|%15s%5s %15s%5s%20s\n",
        "X", "", "Y", "", "|");

    for (int i = 0; i < x.size(); i++) {
        printf("|%20f %20f%20s\n",
            x[i], y[i], "|");
    }

    for (int i = 0; i < 62; i++) {
        printf("_");
    }
    printf("\n");
}

// Function to predict the value
// correspondng to some input
float regression::predict(float x)
{
    return coeff * x + constTerm;
}

// Function to revert the value
// correspondng to some input
float regression::revert(float y)
{
    return (y-constTerm) / coeff;
}

// Function that returns overall
// sum of square of errors
float regression::errorSquare()
{
    float ans = 0;
    for (int i = 0;
        i < x.size(); i++) {
        ans += ((predict(x[i]) - y[i])
                * (predict(x[i]) - y[i]));
    }
    return ans;
}

// Functions that return the error
// i.e the difference between the
// actual value and value predicted
// by our model
float regression::errorIn(float num)
{
    for (int i = 0;
        i < x.size(); i++) {
        if (num == x[i]) {
            return (y[i] - predict(x[i]));
        }
    }
    return 0;
}

