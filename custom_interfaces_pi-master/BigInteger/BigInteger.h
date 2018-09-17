using namespace std;
class BigInteger{
public:
    void setNumber(string number);
    string getNumber() const;
private:
    char sign;
    int length;
    int value[100];
friend BigInteger add( const BigInteger& a, const BigInteger& b);
};
BigInteger add( const BigInteger& a, const BigInteger& b);