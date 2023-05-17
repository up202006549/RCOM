/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1
#define Flag 0x5c
#define A_Transmitter 0x01
#define A_Receiver 0x03
#define C_Set 0x03
#define C_Disc 0x0B
#define C_UA 0x07

volatile int STOP=FALSE;

typedef enum{
	start,
	flagrcv,
	arcv,
	crcv,
	bccrcv,
	stop,
} statenames_estados;


statenames_estados currentState_estados = start;

/*void atende(){
    printf("1\n");                   // atende alarme
    res=write(fd,buf,255);
    printf("rewriting %d bytes\n", res);
    return 0;
}*/



int main(int argc, char** argv)
{
    int fd,c, res;
    char buf[255];
    struct termios oldtio,newtio;
    char buf2[255];
    char buf3[255];
    int i, sum = 0, speed = 0;

    //(void) signal(SIGALRM, atende);  // instala rotina que atende interrupcao

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */



    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");


    /*gets(buf);
    for (i = 0; i < 255; i++) {
        buf[i] = 'a';
    }*/

    /*testing*/
    //buf[25] = '\n';

    buf[0] = Flag;
    buf[1] = A_Transmitter;
    buf[2] = C_Set;
    buf[3] = A_Transmitter^C_Set;
    buf[4] = Flag;

    res = write(fd,buf,255);
    printf("%d bytes written\n", res);
    //alarm(3);

    while (currentState_estados != stop) {       
        res = read(fd,buf,1); 
        /*if(res <= 0){
            res = read(fd,buf,1);
        }*/
        //else{
            switch (currentState_estados){
                case start:
                    if (buf[0] == Flag){
                        currentState_estados = flagrcv; 
                        printf("Flag received\n");
                    } 
                    else{
                    printf("Flag failed\n");
                    }
                    break;
                
                case flagrcv:
                    if(buf[0] == A_Receiver){
                        currentState_estados = arcv;
                        printf("Adress received\n");
                    }
                
                    else if (buf[0] == Flag){
                        currentState_estados = flagrcv;
                        printf("Adress failed. Flag received\n"); 
                    }

                    else if (buf[0] != Flag){
                        currentState_estados = start;
                        printf("Adress failed. Restart\n");
                    }
                    break;

                case arcv:
                    if(buf[0] == C_UA){
                        currentState_estados = crcv;
                        printf("Control received\n");
                    }
                
                    else if (buf[0] == Flag){
                        currentState_estados = flagrcv;
                        printf("Control failed. Flag received\n"); 
                    }
                
                    else if (buf[0] != Flag){
                        currentState_estados = start;
                        printf("Control failed. Restart\n");
                    }

                    break;
                
                case crcv:
                    if ((A_Receiver^C_UA) == buf[0]){
                        currentState_estados = bccrcv;
                        printf("BCC received\n");
                    }

                    else if(buf[0] == Flag){
                        currentState_estados = start;
                        printf("BCC failed. Flag received\n");
                    }

                    else if (buf[0] != Flag){
                        currentState_estados = start;
                        printf("BCC failed. Restart\n");
                    }
                
                    break;
                
                case bccrcv:
                    if (buf[0] == Flag) {
                        //alarm(0);
                        currentState_estados = stop;
                        printf("Flag received\n");
                    }

                    else{
                        currentState_estados = start;
                        printf("Flag failed. Restart\n");
                    }
                    break;
                      
            }
            if (currentState_estados == stop) printf("Success\n");
        //}
    }

    /*while(STOP==FALSE){
        res=read(fd,buf3,1);
        strcat(buf2, buf3);                                   2
        printf(":%c:%d\n", buf3[0],res);
        if(buf3[0]=='\0') STOP=TRUE;
    }*/

    //printf("%s\n", buf2); 





    /*
    O ciclo FOR e as instruções seguintes devem ser alterados de modo a respeitar
    o indicado no guião
    */

        sleep(1);
        if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
            perror("tcsetattr");
            exit(-1);
        }


        close(fd);
        return 0;
}
