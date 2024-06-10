#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// int f11(char* s, char* c) {
//     char* p = s;
//     while( *p != '\0') {
//         if (strcmp(p, c) == 0) {
//             return p - s;
//         }
//         p++;
//     }
//     return -1;
// }


void displayStuff(int* data, int size){
    if (size > 0) {
    printf("%d ", *data);
    displayStuff(++data,size - 1);
    printf("%d ",*(--data));
    }
}
int main() {
    int myData[] = {8, 6, 4, 2};
    displayStuff(myData,4);
    return 0;
}
 

 //a).