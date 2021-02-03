// File:          test.cpp
// Date: 26/01/2021
// Description: File containing testing procedures
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

#include "test.h"

void Test::TestVec(){
	vec a = {0.1, 0.2};
	vec b = {0.5, -0.7};
	if(!(a == a)) printf("Error: Vec::operator== doesn't work a.\n");
	if(a == b) printf("Error: Vec::operator== doesn't work b.\n");
	if(a != a) printf("Error: Vec::operator!= doesn't work.\n");
	if(a + b != (vec){0.6, -0.5}) printf("Error: Vec::operator+ doesn't work.\n");
	if(a - b != (vec){-0.4, 0.9}) printf("Error: Vec::operator- doesn't work.\n");
	if(a * b != -0.09) printf("Error: Vec::operator* doesn't work a; %f\n", a * b);
	if(a * 2 != (vec){0.2, 0.4}) printf("Error: Vec::operator* doesn't work b.\n");
	if(2 * a != (vec){0.2, 0.4}) printf("Error: Vec::operator* doesn't work c.\n");
	if(a / 2 != (vec){0.05, 0.1}) printf("Error: Vec::operator/ doesn't work.\n");
	vec c = a + b;
	a += b;
	if(a != c) printf("Error: Vec::operator+= doesn't work b.\n");
}
