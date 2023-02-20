#pragma GCC optimize("O2,unroll-loops")
#pragma GCC target("avx2,bmi,bmi2,lzcnt,popcnt")

#include <iostream>
#include <exception>
#include <memory>

#if DEBUG
void* operator new(size_t size)
{
	std::cout << "Allocating " << size << " bytes\n";
	return malloc(size);
}

void operator delete(void* memory, size_t size)
{
	std::cout << "Freeing " << size << " bytes\n";
	free(memory);
}
#endif

#define SINGLY_LINKED_LIST true
#define DOUBLY_LINKED_LIST !SINGLY_LINKED_LIST

template <typename T>
struct Node
{
	T m_Data;
#if SINGLY_LINKED_LIST
	Node *ptr_Next;
	Node() : ptr_Next(nullptr) {};
	Node(const T& data) : m_Data(data), ptr_Next(nullptr) {};
#else   // DOUBLY_LINKED_LIST
	Node *ptr_Prev, *ptr_Next;
	Node() : ptr_Prev(nullptr), ptr_Next(nullptr) {};
	Node(const T& data) : m_Data(data), ptr_Prev(nullptr), ptr_Next(nullptr) {};
#endif  // SINGLY_LINKED_LIST
};

template <typename T>
class LinkedList
{
private:
	static void delete_node(Node<T> *node);

public:
	LinkedList() : ptr_Head(nullptr), ptr_Tail(nullptr), m_size(0) {};
	~LinkedList() { delete_node(ptr_Head); };
	void clear();

	Node<T>* find(const T& key) const;
	bool contain(const T& key) { return find(key) != nullptr; };
	size_t get_size() { return m_size; };

	T get_head() const;
	T get_tail() const;
	void display() const;

	void insert_begin(const T& key);
	void insert_end(const T& key);
	void insert(const T& prev_key, const T& key);

    void delete_begin();
	void delete_end();
	void delete_key(const T& key);

private:
	Node<T> *ptr_Head, *ptr_Tail;
	size_t m_size;
};

template <typename T>
void LinkedList<T>::delete_node(Node<T> *node)
{
	if (node == nullptr)
		return;
	Node<T> *next = node->ptr_Next;
	delete node;
	delete_node(next);
}

template <typename T>
Node<T>* LinkedList<T>::find(const T& key) const
{
	Node<T> *ptr = ptr_Head;
	while (ptr != nullptr && ptr->m_Data != key) {
		ptr = ptr->ptr_Next;
	}
	return ptr;
}

template <typename T>
T LinkedList<T>::get_head() const
{
	if (ptr_Head == nullptr) {
		std::cout << "ERROR: Empty linked list!\n";
		throw std::exception();
	}
	return ptr_Head->m_Data;
}

template <typename T>
T LinkedList<T>::get_tail() const
{
	if (ptr_Tail == nullptr) {
		std::cout << "ERROR: Empty linked list!\n";
		throw std::exception();
	}
	return ptr_Tail->m_Data;
}

template <typename T>
void LinkedList<T>::clear()
{
	delete_node(ptr_Head);
	ptr_Head = nullptr;
	ptr_Tail = nullptr;
	m_size = 0;
}

template <typename T>
void LinkedList<T>::display() const
{
	Node<T> *ptr = ptr_Head;
	while (ptr != nullptr) {
		std::cout << ptr->m_Data << " ";
		ptr = ptr->ptr_Next;
	}
	std::cout << "\n";
}

template <typename T>
void LinkedList<T>::insert_begin(const T& key)
{
	if (m_size == 0) {
		ptr_Head = new Node<T>(key);
		ptr_Tail = ptr_Head;
	}	else {
		Node<T> *old_head = ptr_Head;
		ptr_Head = new Node<T>(key);
		ptr_Head->ptr_Next = old_head;
#if DOUBLY_LINKED_LIST
        old_head->ptr_Prev = ptr_Head;
#endif  // DOUBLY_LINKED_LIST
	}
	m_size++;
}

template <typename T>
void LinkedList<T>::insert_end(const T& key)
{
	if (m_size == 0)
		return insert_begin(key);
	ptr_Tail->ptr_Next = new Node<T>(key);
#if DOUBLY_LINKED_LIST
    ptr_Tail->ptr_Next->ptr_Prev = ptr_Tail;
#endif  // DOUBLY_LINKED_LIST
	ptr_Tail = ptr_Tail->ptr_Next;
	m_size++;
}

template <typename T>
void LinkedList<T>::insert(const T& prev_key, const T& key)
{
	Node<T> *ptr = find(prev_key);
	if (ptr == nullptr)
		return;
	if (ptr == ptr_Head)
		return insert_begin(key);
	if (ptr == ptr_Tail)
		return insert_end(key);	
	
	Node<T> *new_node = new Node<T>(key);
	new_node->ptr_Next = ptr->ptr_Next;
#if DOUBLY_LINKED_LIST
    new_node->ptr_Prev = ptr;
    ptr->ptr_Next->ptr_Prev = new_node;
#endif  // DOUBLY_LINKED_LIST
	ptr->ptr_Next = new_node;
	m_size++;
}

template <typename T>
void LinkedList<T>::delete_begin()
{
	if (m_size == 0)
		return;
	if (m_size == 1) {
		delete ptr_Head;
		ptr_Head = nullptr;
		ptr_Tail = nullptr;
	}	else {
		Node<T> *ptr = ptr_Head->ptr_Next;
		delete ptr_Head;
		ptr_Head = ptr;
#if DOUBLY_LINKED_LIST
        ptr_Head->ptr_Prev = nullptr;
#endif  // DOUBLY_LINKED_LIST
	}
	m_size--;
}

template <typename T>
void LinkedList<T>::delete_end()
{
	if (m_size == 0)
		return;
	if (m_size == 1) {
		delete ptr_Head;
		ptr_Head = nullptr;
		ptr_Tail = nullptr;
	}	else {
#if SINGLY_LINKED_LIST
		ptr_Tail = ptr_Head;
		for (size_t i = 0; i < m_size - 2; i++) {
			ptr_Tail = ptr_Tail->ptr_Next;
		}
		delete ptr_Tail->ptr_Next;
		ptr_Tail->ptr_Next = nullptr;
#else   // DOUBLY_LINKED_LIST
        Node<T> *ptr = ptr_Tail->ptr_Prev;
		delete ptr_Tail;
		ptr_Tail = ptr;
        ptr_Tail->ptr_Next = nullptr;
#endif  // SINGLY_LINKED_LIST
	}
	m_size--;
}

template <typename T>
void LinkedList<T>::delete_key(const T& key)
{
	if (m_size == 0)
		return;
#if SINGLY_LINKED_LIST
	if (ptr_Head->m_Data == key)
		return delete_begin();
	Node<T> *ptr1 = ptr_Head;
	Node<T> *ptr2 = ptr_Head->ptr_Next;
	while (true) {
		if (ptr2 == nullptr)
			return;
		if (ptr2->m_Data == key)
			break;
		ptr1 = ptr2;
		ptr2 = ptr2->ptr_Next;
	}
	if (ptr2 == ptr_Tail)
		return delete_end();
	ptr1->ptr_Next = ptr2->ptr_Next;
	delete ptr2;
#else   // DOUBLY_LINKED_LIST
	Node<T> *ptr = find(key);
	if (ptr == ptr_Head)
		return delete_begin();
    if (ptr == ptr_Tail)
        return delete_end();
    ptr->ptr_Prev->ptr_Next = ptr->ptr_Next;
    ptr->ptr_Next->ptr_Prev = ptr->ptr_Prev;
    delete ptr;
#endif  // SINGLY_LINKED_LIST
	m_size--;
}

int main()
{
    LinkedList<int> ll;
    ll.insert_begin(7);
    ll.insert_begin(2);
    ll.insert_end(5);
    ll.insert(6, 9);
    ll.insert(7, 9);
    ll.display();

    ll.delete_key(7);
    ll.display();

    return 0;
}
