#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <iostream>

#include "include/UDP_Com.h"

int main(){

    printf("Server virker\n");

    UDP_Com TEST;
    TEST.ToggleDebug(true);
    
    TEST.ReceiveMessage();
    //TEST.PrintMessage();
}




