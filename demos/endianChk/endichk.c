#include <stdio.h>

union mem {
    unsigned char   byte[4];
    unsigned short  half[2];
    unsigned int    word;
} data;

int main(int argc, char **argv) {
    data.word = (unsigned int)0x12345678;	
    printf("0x08%x: w=0x%08x\n", (unsigned int)&data.word, data.word);
    
    printf("0x08%x: h[0]=0x%04x\n", (unsigned int)&data.half[0], data.half[0]);
    printf("0x08%x: h[1]=0x%04x\n", (unsigned int)&data.half[1], data.half[1]);
    
    printf("0x08%x: b[0]=0x%02x\n", (unsigned int)&data.byte[0], data.byte[0]);
    printf("0x08%x: b[1]=0x%02x\n", (unsigned int)&data.byte[1], data.byte[1]);
    printf("0x08%x: b[2]=0x%02x\n", (unsigned int)&data.byte[2], data.byte[2]);
    printf("0x08%x: b[3]=0x%02x\n", (unsigned int)&data.byte[3], data.byte[3]);
}
