#include "linklayer.h"
#include <time.h>
#define Flag 0x5c
#define A_Transmitter 0x01
#define A_Receiver 0x03
#define C_Set 0x03
#define C_Disc 0x0B
#define C_UA 0x07
#define C_RR1 0x21
#define C_RR0 0x01
#define C_I0 0x00
#define C_I1 0x02
#define C_REJ1 0x24
#define C_REJ0 0x04
#define Esc_char 0x5d
#define Esc_charBCC 0x20


int fd;
int counttimeouts = 0;
int aux = 0;
int sendnumb = 0;
int rcvnumb = 0;
struct termios oldtio,newtio;
int max_timeouts = 0;
int timeout = 0;
time_t start_time, end_time;
int total_trans = 0;
int total_retrans = 0;
int total_timeouts = 0;
int total_trans_data = 0;
int total_RR = 0;
int total_RR_dup_sent = 0;
int total_REJ = 0;
int total_bytes_read = 0;
int total_bytes_sent = 0;
int repetido = 0;

typedef enum{
	start,
	flagrcv,
	arcv,
	crcv,
	bccrcv,
	stop,
    disc,
    dadosrcv,
    bcc2rcv,
} statenames_estados;


void stuffing(unsigned char *buf, int *leng){
    for(int i = 4; i < (*leng)-2; i++){
        if((buf[i] == Flag) || (buf[i] == Esc_char)){
            for(int k = (*leng); k > i; k--){
                buf[k] = buf[k-1];
            }
            buf[i+1] = (buf[i]^Esc_charBCC);
            buf[i] = Esc_char;

            (*leng) = (*leng) + 1;
            i++;
        } 
    }
}

void destuffing(unsigned char *buf, int *leng){
    for(int i = 4; i < (*leng-3); i++){
        if((buf[i] == Esc_char) && ((buf[i+1] == (Flag^Esc_charBCC)) || (buf[i+1] == (Esc_char^Esc_charBCC)))){
            if(buf[i+1] == (Flag^Esc_charBCC)){
                buf[i] = Flag;
            }
            else{
                buf[i] = Esc_char;
            }
            for(int k = i+1; k < (*leng); k++){
                buf[k] = buf[k+1];
            }
            (*leng) = (*leng) -1;
        }
    }
}

int llopen(linkLayer connectionParameters){
    time(&start_time);
    char buf[255];
    int res = 0;

    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY );
        if (fd < 0) { perror(connectionParameters.serialPort); exit(-1); }

        if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
            perror("tcgetattr");
            exit(-1);
        }

        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;

        /* set input mode (non-canonical, no echo,...) */
        newtio.c_lflag = 0;

        newtio.c_cc[VTIME]    = connectionParameters.timeOut/0.1;   /* inter-character timer unused */
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

        //printf("New termios structure set\n");

        //Start 
        statenames_estados currentState_estados;
        currentState_estados = start;
        int max_timeouts = connectionParameters.numTries;
        int timeout = connectionParameters.timeOut;
        //Transmitter
        if(connectionParameters.role == 0){  // 0 -> Transmitter
            
            //Sending SET  

            buf[0] = Flag;
            buf[1] = A_Transmitter;
            buf[2] = C_Set;
            buf[3] = A_Transmitter^C_Set;
            buf[4] = Flag;

            res = write(fd,buf, 5);
            total_trans ++;
            printf("%d bytes written\n", res);

            //Reading UA

            while (currentState_estados != stop) {  

                res = read(fd,buf,1); 
                if(res == 0){
                    counttimeouts ++;
                    currentState_estados = start;
                    if (counttimeouts == max_timeouts){
                        printf("Max timeouts reached UA\n");
                        perror("Timeout");
                        return -1;
                    }

                    buf[0] = Flag;
                    buf[1] = A_Transmitter;
                    buf[2] = C_Set;
                    buf[3] = A_Transmitter^C_Set;
                    buf[4] = Flag;
                    res=write(fd,buf, 5);
                    total_trans ++;
                    total_retrans++;
                    total_timeouts ++;
                    printf("Rewriting SET %d bytes\n", res);   
                }
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
                            if ((A_Transmitter^C_UA) == buf[0]){
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
                                counttimeouts = 0;
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
                if (currentState_estados == stop) printf("Success reading UA\n");
            }
            return 1;
        }

        else if(connectionParameters.role == 1){  // Receiver
            
            //Reading for SET

            while (currentState_estados != stop) {       
                res = read(fd,buf,1); 
                if(res == 0){
                    currentState_estados == start;
                    counttimeouts ++;
                    if(counttimeouts == max_timeouts){
                        perror("Timeout");
                        return -1;
                    }
                    total_timeouts ++;
                }
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
                if (currentState_estados == stop) printf("Success reading SET\n");
            }
            //Sending UA
            buf[0] = Flag;
            buf[1] = A_Transmitter;
            buf[2] = C_UA;
            buf[3] = A_Transmitter^C_UA;
            buf[4] = Flag;

            printf("Writing UA: %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
            res = write(fd,buf, 5);
            total_trans++;
            printf("%d Bytes written \n", res);
            return 1;
        }

}

int llwrite(char* buf, int bufSize){

    //Sending I
    statenames_estados currentState_estados;
    unsigned char send[MAX_PAYLOAD_SIZE*2 + 7];
    unsigned char bcc2[1];
    unsigned char rcv[MAX_PAYLOAD_SIZE*2 + 7];
    int sendleng = 6;
    int res = 0;
    bcc2[0] = buf[0];

    for(int i = 1; i < bufSize; i++){
        bcc2[0] = (bcc2[0]^buf[i]);
    }

    send[0] = Flag;
    send[1] = A_Transmitter;

    if (sendnumb == 0){
        send[2] = C_I0;
        send[3] = A_Transmitter^C_I0;
  
    }
    else if (sendnumb == 1){
        send[2] = C_I1;
        send[3] = A_Transmitter^C_I1;

    }
    else{
        return -1;
    }

    for(int i = 0; i < (bufSize); i++){
        send[4+i] = buf[i];
        sendleng ++;
    }

    send[sendleng - 2] = bcc2[0];
    send[sendleng - 1] = Flag; 

    stuffing(send, &sendleng);

    printf("Writing I%d %02x %02x %02x %02x %02x %02x %02x %02x\n", sendnumb, send[0], send[1], send[2], send[3], send[4], send[sendleng - 3], send[sendleng - 2], send[sendleng - 1]);

    if((write(fd, send, sendleng)) != sendleng){
        return -1;
    }
    total_trans ++;
    total_trans_data++;
    //waiting for RR
	int lengrcv;

    currentState_estados = start;
    while (currentState_estados != stop) {  

        res = read(fd,buf,1); 
        if(res == 0){
            counttimeouts ++;
            currentState_estados = start;
            if (counttimeouts == max_timeouts){
                printf("Max timeouts reached RR\n");
                perror("Timeout");
                return -1;
            }
            res=write(fd, send, sendleng);
            total_trans ++;
            total_retrans ++;
            total_trans_data ++;
            total_timeouts ++;
            printf("Rewriting %d bytes\n", res);
        }
        if(res > 0){
            printf("???????write %02x\n",buf[0]);
            switch (currentState_estados){
                case start:
                    lengrcv = 0;
                    if (buf[0] == Flag){
                        rcv[0] = buf[0];
                        lengrcv ++;
                        currentState_estados = flagrcv; 
                        printf("Flag received %02x %d\n", rcv[lengrcv-1], lengrcv);
                    } 
                    else{
                    printf("Flag failed\n");
                    }
                    break;
                
                case flagrcv:
                    if(buf[0] == A_Transmitter){
                        rcv[lengrcv] = buf[0];
                        lengrcv ++;
                        currentState_estados = arcv;
                        printf("Adress received %02x %d\n", rcv[lengrcv-1], lengrcv);
                    }
                
                    else if (buf[0] == Flag){
                        lengrcv = 0;
                        rcv[lengrcv] = buf[0]; 
                        lengrcv++;
                        currentState_estados = flagrcv; 
                        printf("Adress failed flag received %02x %d\n", rcv[lengrcv-1], lengrcv); 
                    }

                    else if (buf[0] != Flag){
                        currentState_estados = start;
                        printf("Adress failed. Restart\n");
                    }
                    break;

                case arcv:
                    if((buf[0] == C_RR1) || (buf[0] == C_RR0) || (buf[0] == C_REJ0) || (buf[0] == C_REJ1)){
                        rcv[lengrcv] = buf[0];
                        lengrcv ++;
                        currentState_estados = crcv;
                        printf("Control received %02x %d\n", rcv[lengrcv-1], lengrcv);
                    }
                
                    else if (buf[0] == Flag){
                        lengrcv = 0;
                        rcv[lengrcv] = buf[0];
                        lengrcv++; 
                        currentState_estados = flagrcv; 
                        printf("Control failed flag received %02x %d\n", rcv[lengrcv-1], lengrcv); 
                    }
                
                    else if (buf[0] != Flag){
                        currentState_estados = start;
                        printf("Control failed. Restart\n");
                    }

                    break;
                
                case crcv:
                    if (buf[0] == (rcv[1]^rcv[2])){
                        rcv[lengrcv] = buf[0];
                        lengrcv ++;                            
                        currentState_estados = bccrcv;
                        printf("BCC received %02x %d\n", rcv[lengrcv-1], lengrcv);
                    }

                    else if(buf[0] == Flag){
                        lengrcv = 0;
                        rcv[lengrcv] = buf[0]; 
                        lengrcv++;
                        currentState_estados = flagrcv; 
                        printf("BCC failed flag received %02x %d\n", rcv[lengrcv-1], lengrcv);
                    }

                    else if (buf[0] != Flag){
                        currentState_estados = start;
                        printf("BCC failed. Restart\n");
                    }
                
                    break;
                
                case bccrcv:
                    if (buf[0] == Flag) {
                        if((rcv[2] == C_REJ0) || (rcv[2] == C_REJ1)) {
                            currentState_estados = start;
                            counttimeouts = 0;
                            res=write(fd, send, sendleng);
                            total_trans ++;
                            total_retrans ++;
                            total_REJ++;
                            printf("Rewriting %d bytes\n", res);
                            break;
                        }
                        counttimeouts = 0;
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
    }
    if(rcv[2] == C_RR1){
            sendnumb = 1;
            total_RR++;
        }
                        
        else if(rcv[2] == C_RR0){
            sendnumb = 0;
            total_RR++;
        }
    printf("Success reading RR\n");
    total_bytes_sent += bufSize;
    return bufSize;
    
}

int llread(char* packet){
    int done = 0;
    int res = 0;
    statenames_estados currentState_estados;
    unsigned char rcv[MAX_PAYLOAD_SIZE*2 + 7];
    unsigned char dados[MAX_PAYLOAD_SIZE*2];
    unsigned char buf[7];
    unsigned char bcc2[1];
    unsigned char CRR[1];
    int SET = 0;
    int RR = 0;
    int lengrcv = 0;
    int lengdados = 0;
    int bcc2pos;

    //Reading I
    currentState_estados = start;
    while(done != 1){
        while (currentState_estados != stop) {       
            res = read(fd,buf,1); 
            if(res == 0){
                currentState_estados == start;
                counttimeouts ++;
                if(counttimeouts == max_timeouts){
                    perror("Timeout");
                    return -1;
                }
                total_timeouts ++;
            }
            if(res > 0){
                printf("???????read %02x\n", buf[0]);
                switch (currentState_estados){
                    case start:
                        if (buf[0] == Flag){
                            SET = 0;
                            RR = 0;
                            lengrcv = 0;
                            lengdados = 0;
                            bcc2pos;
                            rcv[lengrcv] = buf[0]; 
                            lengrcv ++;
                            currentState_estados = flagrcv; 
                            printf("Flag received %02x %d\n", rcv[lengrcv-1], lengrcv);
                        } 
                        else{
                            printf("Flag failed\n");
                        }
                        break;
                        
                    case flagrcv:
                        if(buf[0] == A_Transmitter){
                            rcv[lengrcv] = buf[0];
                            lengrcv ++;
                            currentState_estados = arcv;
                            printf("Adress received %02x %d\n", rcv[lengrcv-1], lengrcv);
                        }
                        
                        else if (buf[0] == Flag){
                            lengdados = 0;
                            lengrcv = 0;
                            rcv[lengrcv] = buf[0];
                            lengrcv++; 
                            currentState_estados = flagrcv; 
                            printf("Flag received %02x %d\n", rcv[lengrcv-1], lengrcv);
                        }

                        else if (buf[0] != Flag){
                            currentState_estados = start;        
                            printf("Adress failed. Restart\n");
                        }
                        break;    

                    case arcv:
                        if((buf[0] == C_I1) || (buf[0] == C_I0) || (buf[0] == C_Set)){
                            if((buf[0] == C_I0) && (rcvnumb == 1)){
                                repetido = 1; 
                                CRR[0] = C_RR1;
                            }
                                
                            else if((buf[0] == C_I1) && (rcvnumb == 0)){
                                repetido = 1;
                                CRR[0] = C_RR0;
                            }

                            else if(buf[0] == C_I0){
                            CRR[0] = C_RR1; 
                            }
                            
                            else if(buf[0] == C_I1){
                            CRR[0] = C_RR0;
                            }

                            else if(buf[0] == C_Set){
                                SET = 1;
                            }
                            rcv[lengrcv] = buf[0];
                            lengrcv ++;
                            currentState_estados = crcv;
                            printf("Control received %02x %d\n", rcv[lengrcv-1], lengrcv);
                        }
                            
                        else if (buf[0] == Flag){
                            lengdados = 0;
                            lengrcv = 0;
                            rcv[lengrcv] = buf[0]; 
                            lengrcv++;
                            currentState_estados = flagrcv; 
                            printf("Flag received %02x %d\n", rcv[lengrcv-1], lengrcv);
                        }
                            
                        else if (buf[0] != Flag){
                            currentState_estados = start;
                            printf("Control failed. Restart\n");
                        }    

                        break;
                            
                    case crcv:
                        if (((A_Transmitter^C_I0) == buf[0]) || ((A_Transmitter)^C_I1 == buf[0]) || ((A_Transmitter)^C_Set == buf[0])  ){
                            rcv[lengrcv] = buf[0];
                            lengrcv ++;                            
                            currentState_estados = bccrcv;
                            printf("BCC1 received %02x %d\n", rcv[lengrcv-1], lengrcv);
                        }

                        else if(buf[0] == Flag){
                            lengdados = 0;
                            lengrcv = 0;
                            rcv[lengrcv] = buf[0]; 
                            lengrcv++;
                            currentState_estados = flagrcv; 
                            printf("Flag received %02x %d\n", rcv[lengrcv-1], lengrcv);
                        }

                        else if (buf[0] != Flag){
                            currentState_estados = start;
                            printf("BCC1 failed. Restart\n");
                        }
                            
                        break;
                            
                    case bccrcv:
                        
                        if((SET == 1) && (buf[0] == Flag) ){
                            buf[0] = Flag;
                            buf[1] = A_Transmitter;
                            buf[2] = C_UA;
                            buf[3] = A_Transmitter^C_UA;
                            buf[4] = Flag;

                            printf("Writing UA: %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
                            res = write(fd,buf, 5);
                            printf("%d Bytes written \n", res);
                            currentState_estados = start;
                            total_trans ++;
                            total_retrans ++;
                            break;
                        }  
                        rcv[lengrcv] = buf[0];
                        lengrcv ++;

                        if(buf[0] == Flag){

                            currentState_estados = stop;
                        }

                        break;                       
                }
            }
            if (currentState_estados == stop) printf("Finished reading I%d\n", sendnumb);
        }
        destuffing(rcv, &lengrcv);
        total_trans_data ++;
        bcc2pos = lengrcv - 2;
        lengdados = lengrcv - 6;
        total_bytes_read += lengdados;
        bcc2[0] = rcv[4];
        for(int i = 5; i < lengrcv-2; i++){
            bcc2[0] = (bcc2[0]^rcv[i]);  
        }
        printf("\tBCC2 %02x\n", bcc2[0]);
        if(bcc2[0] == rcv[bcc2pos]){

            if((rcv[2] == C_I0) && (rcvnumb == 0)){
                rcvnumb = 1;
            }
                            
            else if((rcv[2] == C_I1) && (rcvnumb == 1)){
                rcvnumb = 0;
            }

            for(int i = 0; i < lengdados; i++){
                packet[i] = rcv[4+i];  
            }

            buf[0] = Flag;
            buf[1] = A_Transmitter;
            buf[2] = CRR[0];
            buf[3] = A_Transmitter^CRR[0];
            buf[4] = Flag;
            printf("Writing RR%d: %02x %02x %02x %02x %02x\n", rcvnumb, buf[0], buf[1], buf[2], buf[3], buf[4]);
            res = write(fd,buf, 5);
            total_trans ++;
            total_RR ++;
            if(repetido == 0){
               done = 1; 
            }
            else{
                total_RR_dup_sent ++;
                repetido = 0;
            }                  
        }

        else if(repetido == 1){
            buf[0] = Flag;
            buf[1] = A_Transmitter;
            buf[2] = CRR[0];
            buf[3] = A_Transmitter^CRR[0];
            buf[4] = Flag;
            printf("Writing RR%d: %02x %02x %02x %02x %02x\n", rcvnumb, buf[0], buf[1], buf[2], buf[3], buf[4]);
            res = write(fd,buf, 5);
            done = 1; 
            total_trans ++;
            total_RR_dup_sent++;
            total_RR ++;  
            repetido = 0;       
                
        }
                            
        else if(CRR[0] == C_RR1){ // Recebi I0
            buf[0] = Flag;
            buf[1] = A_Transmitter;
            buf[2] = C_REJ0;
            buf[3] = A_Transmitter^C_REJ0;
            buf[4] = Flag;
            printf("Writing REJ%d: %02x %02x %02x %02x %02x\n", sendnumb, buf[0], buf[1], buf[2], buf[3], buf[4]);
            res = write(fd,buf, 5);
            currentState_estados = start; 
            total_retrans++;
            total_trans ++;
            total_REJ ++;
        }

        else if(CRR[0] == C_RR0){ //Recebi I1
            buf[0] = Flag;
            buf[1] = A_Transmitter;
            buf[2] = C_REJ1;
            buf[3] = A_Transmitter^C_REJ1;
            buf[4] = Flag;
            printf("Writing REJ%d: %02x %02x %02x %02x %02x\n", sendnumb, buf[0], buf[1], buf[2], buf[3], buf[4]);
            res = write(fd,buf, 5);                        
            currentState_estados = start; 
            total_retrans++;
            total_trans ++;
            total_REJ ++;
        } 
    }
    return lengdados;   
}

int llclose(linkLayer connectionParameters, int showStatistics){
    statenames_estados currentState_estados;
    currentState_estados = start;
    int res = 0;
    unsigned char buf[7];

    //Transmitter
    if(connectionParameters.role == 0){  // 0 -> Transmitter
        
        //Sending Disc

        buf[0] = Flag;
        buf[1] = A_Transmitter;
        buf[2] = C_Disc;
        buf[3] = A_Transmitter^C_Disc;
        buf[4] = Flag;

        res = write(fd,buf,5);
        total_trans ++;
        printf("%d bytes written\n", res);

        //Reading Disc

        while (currentState_estados != stop) { 

            res = read(fd,buf,1); 
            if(res == 0){
                counttimeouts ++;
                currentState_estados = start;
                if (counttimeouts == max_timeouts){
                    printf("Max timeouts reached Disc\n");
                    perror("Timeout");
                    return -1;
                }

                buf[0] = Flag;
                buf[1] = A_Transmitter;
                buf[2] = C_Disc;
                buf[3] = A_Transmitter^C_Disc;
                buf[4] = Flag;
                res=write(fd,buf, 5);
                total_trans ++;
                total_retrans++;
                total_timeouts ++;
                printf("Rewriting SET %d bytes\n", res);
            }
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
                        if(buf[0] == C_Disc){
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
                        if ((A_Receiver^C_Disc) == buf[0]){
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
                            counttimeouts = 0;
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
            if (currentState_estados == stop) printf("Success reading Disc\n");
        }

        // Sending UA

        buf[0] = Flag;
        buf[1] = A_Receiver;
        buf[2] = C_UA;
        buf[3] = A_Receiver^C_UA;
        buf[4] = Flag;

        printf("Writing UA: %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
        res = write(fd,buf, 5);~
        total_trans ++;
        printf("%d Bytes written \n", res);

        time(&end_time);
        if(showStatistics == TRUE){
                printf("STATISTICS ------------------------------------------------------------------\n");
                printf("General Statistics Transmitter\n");
                printf("Total Time: %.2lf s\n", difftime(end_time, start_time));
                printf("Total Transmissions: %d\n", total_trans);
                printf("Total Re-transmissions: %d\n", total_retrans);
                printf("Total Number of TimeOuts: %d\n", total_timeouts);
                printf("Data Statistics\n");
                printf("Total Transmissions Sent: %d\n", total_trans_data);
                printf("Total Number of RR Received: %d\n", total_RR);
                printf("Total Number of REJ Received: %d\n", total_REJ);
                printf("Total Bytes Sent: %d\n", total_bytes_sent);
                printf("Success Rate: %.2f%%\n", (float)((total_trans - total_retrans)*100) / total_trans);
                printf("Data Success Rate: %.2f%%\n", (float)(total_RR*100) / total_trans_data);
            }

    }

    else if(connectionParameters.role == 1){  // Receiver
        
        //Reading Disc

        while (currentState_estados != stop) {       
            res = read(fd,buf,1); 
            if(res == 0){
                currentState_estados == start;
                counttimeouts ++;
                if(counttimeouts == max_timeouts){
                    perror("Timeout");
                    return -1;
                }
                total_timeouts ++;
            }
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
                        if(buf[0] == C_Disc){
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
                        if ((A_Transmitter^C_Disc) == buf[0]){
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
            if (currentState_estados == stop) printf("Success reading Disc\n");
        }
        //Sending Disc
        buf[0] = Flag;
        buf[1] = A_Receiver;
        buf[2] = C_Disc;
        buf[3] = A_Receiver^C_Disc;
        buf[4] = Flag;

        printf("Writing Disc: %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
        res = write(fd,buf, 5);
        total_trans ++;
        printf("%d Bytes written \n", res);

        // Reading UA
        while (currentState_estados != stop) { 
            res = read(fd,buf,1); 
            if(res == 0){
                    counttimeouts ++;
                    currentState_estados = start;
                    if (counttimeouts == max_timeouts){
                        printf("Max timeouts reached UA\n");
                        perror("Timeout");
                        return -1;
                    }

                    buf[0] = Flag;
                    buf[1] = A_Receiver;
                    buf[2] = C_Disc;
                    buf[3] = A_Receiver^C_Disc;
                    buf[4] = Flag;
                    res=write(fd,buf, 5);
                    total_trans ++;
                    total_retrans++;
                    total_timeouts ++;
                    printf("Rewriting SET %d bytes\n", res);   
            }
            if (res > 0){
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
                        if ((A_Transmitter^C_UA) == buf[0]){
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
                            counttimeouts = 0;
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
            if (currentState_estados == stop) printf("Success reading UA\n");
        }
        time(&end_time);
        if(showStatistics == TRUE){
                printf("STATISTICS ------------------------------------------------------------------\n");
                printf("General Statistics Receiver\n");
                printf("Total Time: %.2lf s\n", difftime(end_time, start_time));
                printf("Total Transmissions: %d\n", total_trans);
                printf("Total Re-transmissions: %d\n", total_retrans);
                printf("Total Number of TimeOuts: %d\n", total_timeouts);
                printf("Data Statistics\n");
                printf("Total Transmissions Received: %d\n", total_trans_data);
                printf("Total Number of RR Sent: %d\n", total_RR);
                printf("Total Number of RR Duplicates Sent: %d\n", total_RR_dup_sent);
                printf("Total Number of REJ Sent: %d\n", total_REJ);
                printf("Total Bytes Read: %d\n", total_bytes_read);
                printf("Success Rate: %.2f%%\n", (float)((total_trans - total_retrans)*100) / total_trans);
                printf("Data Success Rate: %.2f%%\n", (float)(total_RR*100) / total_trans_data);
            }
    }


    sleep( 1 + connectionParameters.timeOut );

    if (tcsetattr(fd,TCSANOW,&oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    return 0;

}