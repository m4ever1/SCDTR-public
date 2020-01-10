#include "../include/defs.h"

struct node
{
    int id;
    node *next;
};

id_list::id_list() //CONSTRUCTOR
{
    head = nullptr;
    tail = nullptr;
    lenght = 0;
}
void id_list::addid(int value)
{
    node *curr=head;
    node *newnode = new node;
    newnode->id = value;
    newnode->next = NULL;
    if(head == NULL)
    {
        head = newnode;
        tail = newnode;
    }
    else
    {
        while(curr->next != NULL)
        {
            curr = curr->next;
        }
        curr->next = newnode;
        tail = newnode;
    }
    lenght++;
}
int id_list::get_lenght()
{     
    return(lenght);    
}
int id_list::get_addr(int addr)
{
    node *temp = head;
    if(addr > lenght)
    {
        return(-1); //ERROR, addr is greater than list size
    }
    for(int i = 0; i < addr; i++)
    {
        temp = temp->next;
    }
    return(temp->id);
}
int* id_list::get_id_array()
{
    int* arrayp = new int[lenght];
    node *temp = head;
    for(int i = 0; i < lenght; i++)
    {
        arrayp[i] = temp->id;
        temp = temp->next;
    }
    return(arrayp);
}
void id_list::delete_last()
{
    node *current=new node;
    node *previous=new node;
    current=head;
    while(current->next!=NULL)
    {
        previous=current;
        current=current->next;	
    }
    tail=previous;
    previous->next=NULL;
    delete current;
}
void id_list::delete_position(int pos)
{
    node *current=new node;
    node *previous=new node;
    current=head;
    for(int i=0;i<pos;i++)
    {
      previous=current;
      current=current->next;
    }
    previous->next=current->next;

}
bool id_list::is_in_list(int id)
{
    node *curr = head;
    for(int i=0; i < lenght; i++)
    {
        if(curr->id == id)
        {
            return(1);
        }
        curr = curr->next;
    }
    return(0);
}
void id_list::swap(int* a, int* b)  
{  
    int t = *a;  
    *a = *b;  
    *b = t;  
} 
int id_list::partition (int arr[], int low, int high)  
{  
    int pivot = arr[high]; // pivot  
    int i = (low - 1); // Index of smaller element  
  
    for (int j = low; j <= high - 1; j++)  
    {  
        // If current element is smaller than the pivot  
        if (arr[j] < pivot)  
        {  
            i++; // increment index of smaller element  
            swap(&arr[i], &arr[j]);  
        }  
    }  
    swap(&arr[i + 1], &arr[high]);  
    return (i + 1);  
}  
void id_list::quickSort(int arr[], int low, int high)  
{  
    if (low < high)  
    {  
        /* pi is partitioning index, arr[p] is now  
        at right place */
        int pi = partition(arr, low, high);
  
        // Separately sort elements before  
        // partition and after partition  
        quickSort(arr, low, pi - 1);
        quickSort(arr, pi + 1, high);
    }
}
int* id_list::get_sorted_array()
{
    int* arr = get_id_array();
    quickSort(arr, 0, lenght-1);
    return(arr);
};
// int main()
// {
//     id_list list;
//     list.addid(1);
//     list.addid(69);
//     list.addid(102);
//     list.get_lenght();
//     unsigned char* idarr = list.get_id_array();
//     printf("lenght = %d \n",list.get_lenght());
//     for(int i = 0; i < list.get_lenght(); i++)
//     {
//         printf(" array = %d \t", idarr[i]);

//     }
//             printf("\n in? %d \n", list.is_in_list(69));
// }