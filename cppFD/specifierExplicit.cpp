class A
{
public:
    explicit A();
    explicit A(int);
    explicit A(const char*, int = 0);
};

int main()
{
    A a2 = A(1);
    A a3(1);
    A a4 = A("Venditti");
    A a5 = (A)1;
    A a6 = static_cast<A>(1);
    return 0;
}
