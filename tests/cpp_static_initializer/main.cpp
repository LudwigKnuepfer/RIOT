#include "stdio.h"

using namespace std;

struct foo {
    foo() : data(1) { }
    int data;
};

static foo bar;
//static foo bar __attribute__ ((init_priority (101)));

int main() {
    printf("%d\n", bar.data);
    return 0;
}    
