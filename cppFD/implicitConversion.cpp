class A
{
public:
  A();
  A(int);
  A(const char*, int = 0);
};

int main()
{
  // Legal thanks to implicit conversion
  A c = 1;
  A d = "Venditti";
  return 0;
}