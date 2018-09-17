#include <iostream>
#include <string>
#include "BigInteger.h"
using namespace std;

void BigInteger::setNumber(string number){
	if(number[0]=='-'){
        sign = '-';
        number = number.substr(1);
    }else{
        sign = '+';
    }
    length = number.length();
    int j = 0;
    for(int i = number.length()-1; i>=0; i--){
        value[j] = number[i] - '0';
        j++;
    }
}

string BigInteger::getNumber() const{
	string s;
    s = s + (sign == '-'?"-":"");
    for(int j = length - 1; j >= 0; j--)
        s = s + (char)(value[j] + '0');
    return s;
}

BigInteger add( const BigInteger& a, const BigInteger& b){
    BigInteger c;
    if( a.sign == '+' && b.sign == '+'){
        c.sign = '+';
        int carry = 0;
        int i;
        for( i = 0; i< a.length || i< b.length; i++){
            c.value[i] = ((i < a.length?a.value[i]:0) + (i < b.length?b.value[i]:0) + carry) % 10;
            carry = ((i < a.length?a.value[i]:0) + (i < b.length?b.value[i]:0) + carry)/10;
        }
        if(carry > 0){
            c.value[i] = carry;
            i++;
        }
        c.length = i;
        return c;
    }else if(a.sign == '-' && b.sign == '-'){
        a.sign = '+';
        b.sign = '+';

        c = add(a,b);
        c.sign = '-';
        return c;
    }else if(a.sign == '+' && b.sign == '-'){
        int carry = 0;
        int i;
        for(i = 0; i<a.length || i<b.length; i++){
            c.value[i] = ((i < a.length?a.value[i]:0) + 10 - (i < b.length?b.value[i]:0) + carry) % 10;
            carry = ((i < a.length?a.value[i]:0) + 10 - (i < b.length?b.value[i]:0) + carry) / 10 - 1;
        }
        c.length = i;
        while(c.value[c.length - 1] == 0 && c.length > 0){
            c.length --;
        }
        c.length = c.length > 0? c.length:1;
        if(carry == 0){
            c.sign = '+';
        }else{
            BigInteger d;
            d.sign = '+';
            d.length = i + 1;
            for(int j=0; j<i; j++)
                d.value[j] = 0;
            d.value[i] = 1;
            c.sign = '-';
            c = add(d, c);
            c.sign = '-';
        }
        return c;
    }else if(a.sign == '-' && b.sign == '+'){
        return add(b,a);
    }
}


