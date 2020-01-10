#ifndef _LIST_H
#define _LIST_H
#include <stdio.h>

struct node;

class id_list
{
    private:
        node *head, *tail;
        int lenght;
    public:
        id_list(); //CONSTRUCTOR
        void addid(int value);
        int get_addr(int addr);
        int get_lenght();
        int* get_id_array();
        void delete_last();
        void delete_position(int pos);
        bool is_in_list(int id);
        void swap(int* a, int* b);        // FROM https://www.geeksforgeeks.org/selection-sort/
        int partition (int arr[], int low, int high);        // FROM https://www.geeksforgeeks.org/selection-sort/
        void quickSort(int arr[], int low, int high);        // FROM https://www.geeksforgeeks.org/selection-sort/
        int* get_sorted_array();

};

#endif