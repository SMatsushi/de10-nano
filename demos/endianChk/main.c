#include <stdio.h>

union mem {
    unsigned char   byte[8];
    unsigned short  half[4];
    unsigned int    word[2];
    unsigned long   lw;
} longWord;

int main(int argc, char **argv) {
    longWord.lw = (unsigned long)0x1122334455667788;	
    printf("0x%16lx: lw=0x%016lx\n", (unsigned long)&longWord.lw, longWord.lw);

    printf("0x%16lx: w[0]=0x%08x\n", (unsigned long)&longWord.word[0], longWord.word[0]);
    printf("0x%16lx: w[1]=0x%08x\n", (unsigned long)&longWord.word[1], longWord.word[1]);
    
    printf("0x%16lx: h[0]=0x%04x\n", (unsigned long)&longWord.half[0], longWord.half[0]);
    printf("0x%16lx: h[1]=0x%04x\n", (unsigned long)&longWord.half[1], longWord.half[1]);
    printf("0x%16lx: h[2]=0x%04x\n", (unsigned long)&longWord.half[2], longWord.half[2]);
    printf("0x%16lx: h[3]=0x%04x\n", (unsigned long)&longWord.half[3], longWord.half[3]);
    
    printf("0x%16lx: b[0]=0x%02x\n", (unsigned long)&longWord.byte[0], longWord.byte[0]);
    printf("0x%16lx: b[1]=0x%02x\n", (unsigned long)&longWord.byte[1], longWord.byte[1]);
    printf("0x%16lx: b[2]=0x%02x\n", (unsigned long)&longWord.byte[2], longWord.byte[2]);
    printf("0x%16lx: b[3]=0x%02x\n", (unsigned long)&longWord.byte[3], longWord.byte[3]);
    printf("0x%16lx: b[4]=0x%02x\n", (unsigned long)&longWord.byte[4], longWord.byte[4]);
    printf("0x%16lx: b[5]=0x%02x\n", (unsigned long)&longWord.byte[5], longWord.byte[5]);
    printf("0x%16lx: b[6]=0x%02x\n", (unsigned long)&longWord.byte[6], longWord.byte[6]);
    printf("0x%16lx: b[7]=0x%02x\n", (unsigned long)&longWord.byte[7], longWord.byte[7]);
}
