#include <iostream>
#include <stack>

using std::stack;
using std::cout, std::endl;

int main() {
	stack<int> st;

	st.push(21);
	st.push(22);
	st.push(24);
	st.push(25);
	st.push(27);

	while (!st.empty()) {
		cout << st.top() <<" ";
		st.pop();
	}
	cout << endl;

    return 0;
}
