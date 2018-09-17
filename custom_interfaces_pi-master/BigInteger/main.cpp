#include <iostream>
#include <string>
#include "BigInteger.h"
using namespace std;

int main(){
    BigInteger b;
    //input
    cout << "Pls enter a BigInteger value\n";
    string number;
    cin >> number;
    
    b.setNumber(number);
    //output
    BigInteger c;
    cout <<"Another BigInteger: ";
    cin >> number;
    c.setNumber(number);
    BigInteger a;
    a = add(b,c);
    cout << "the result of addition is: ";
    cout << a.getNumber() << endl;
    return 0;
    
}

