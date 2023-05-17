/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#define BAUDRATE B38400
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

int main(int argc, char** argv)
{
   
    
    int fd,c, res;
    struct termios oldtio,newtio;
    unsigned char buf[255];
    unsigned char buf2[255];
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

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0.1;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 0;   /* blocking read until 5 chars received */

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

    while (currentState_estados != stop) {       
        res = read(fd,buf,1); 
        if(res > 0){
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
                    if(buf[0] == A_Transmitter){
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
                    if(buf[0] == C_Set){
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
                    if ((A_Transmitter^C_Set) == buf[0]){
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
                        currentState_estados = stop;
                        printf("Flag received\n");
                    }

                    else{
                        currentState_estados = start;
                        printf("Flag failed. Restart\n");
                    }
                    break;  
                        
            }
        }
        if (currentState_estados == stop) printf("Success\n");
    }

    buf[0] = Flag;
    buf[1] = A_Receiver;
    buf[2] = C_UA;
    buf[3] = A_Receiver^C_UA;
    buf[4] = Flag;

    printf("Writing: %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
    res = write(fd,buf, 255);
    printf("%d Bytes written \n", res);

    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */
    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
