// Modify and refactor from original source:
// https://www.geeksforgeeks.org/merge-sort/
// This code is contributed by Mayank Tyagi
// This code was revised by Joshua Estes

#include <iostream>

using namespace std;

void merge(int a[], const int left, const int mid, const int right)
{
	const int size1 = mid - left + 1;
	const int size2 = right - mid;

	// Create temp arrays
	int array1[size1];
    int array2[size2];
	for (int i = 0; i < size1; i++)
		array1[i] = a[left + i];
	for (int i = 0; i < size2; i++)
		array2[i] = a[mid + 1 + i];

	int id1 = 0;        // id on the 1st subarray
    int id2 = 0;        // id on the 2nd subarray
	int id = left;      // id on the merged array

	// Merge the temp arrays back into array[left..right]
	while (id1 < size1 && id2 < size2) {
		if (array1[id1] <= array2[id2]) {
			a[id] = array1[id1];
			id1++;
		}   else {
			a[id] = array2[id2];
			id2++;
		}
		id++;
	}

	// Copy the remaining elements, if there are any
    for (; id1 < size1; id1++) {
        a[id] = array1[id1];
		id++;
    }
    for (; id2 < size2; id2++) {
        a[id] = array2[id2];
		id++;
    }   
}

void mergeSort(int array[], int const begin, int const end)
{
	if (begin >= end)
		return; // Returns recursively

	int mid = begin + (end - begin) / 2;
	mergeSort(array, begin, mid);
	mergeSort(array, mid + 1, end);
	merge(array, begin, mid, end);
}

void printArray(int A[], int size);

int main()
{
	int arr[] = { 12, 11, 13, 5, 6, 7 };
	auto arr_size = sizeof(arr) / sizeof(arr[0]);
	cout << "Given array is \n";
	printArray(arr, arr_size);

	mergeSort(arr, 0, arr_size - 1);
	cout << "Sorted array is \n";
	printArray(arr, arr_size);
	return 0;
}

void printArray(int A[], int size)
{
	for (auto i = 0; i < size; i++)
		cout << A[i] << " ";
    cout << "\n";
}
