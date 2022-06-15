#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#define IP "192.168.137.167"

typedef struct sockaddr_in sockaddr_in;
typedef struct sockaddr sockaddr;
#define BUFSIZE 256
#define GO_F "go_forward\r\n"
#define GO_B "go_back\r\n"
#define GO_R "go_right\r\n"
#define GO_L "go_left\r\n"
#define GO_S "go_stop\r\n"
#define GO_E "go_end\r\n"


void recv_from_client(int clnt_sock, char * buffer){
	char from_bot[BUFSIZE];
	int i = 0;
	int str_len = read(clnt_sock, from_bot, BUFSIZE);
	while(i < str_len){
		if(from_bot[i] == '\r' && from_bot[i+1] == '\n'){
			buffer[i] = '\0';
			return;
		}
		else buffer[i] = from_bot[i];
		i++;
	}
}

int
main(int argc, char **argv)
{
	int sock;
	sockaddr_in serv_addr;
	sockaddr_in clnt_addr;
	int str_len;
    char buffer[BUFSIZE];

	if((sock = socket(PF_INET, SOCK_STREAM, 0)) == -1){
		perror("socket() error\n");
		exit(1);
	}

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(IP);
	serv_addr.sin_port = htons(50000);

	if(connect(sock, (sockaddr*)&serv_addr, sizeof(serv_addr)) == -1){
		perror("bind() error\n");
		exit(1);
	}
    puts("we are in");
    while(1){
        char c;
        printf("INPUT COMMAND:\n");
        printf("R - readline\nC - calibrate\nG - run bot\nS - stop bot\nM - manual movement\nV - printf pid value\nI - set value\n");
        c = getchar();
        if(c == 'R'){ write(sock, "R\r\n", 3); }
        else if(c == 'C'){ write(sock, "C\r\n", 3); }
        else if(c == 'G'){ write(sock, "G\r\n", 3); }
        else if(c == 'S'){ write(sock, "S\r\n", 3); }
        else if(c == 'V'){ write(sock, "PID\r\n", 5); }
        else if(c == 'M'){
            write(sock, "M\r\n", 3);
            system("/bin/stty raw");
            while(1){
                c = getchar();
                if(c == 'a'){ write(sock, GO_L, strlen(GO_L)); }
                else if(c == 's'){ write(sock, GO_B, strlen(GO_B)); }
                else if(c == 'd'){ write(sock, GO_R, strlen(GO_R)); }
                else if(c == 'w'){ write(sock, GO_F, strlen(GO_F)); }
                else if(c == 'e'){ write(sock, GO_S, strlen(GO_S)); }
                else if(c == 'q'){ write(sock, GO_E, strlen(GO_E)); break; }
            }
            system("/bin/stty cooked");
            recv_from_client(sock, buffer);
            printf("--%s**\n", buffer);
            continue;
        }
        else if(c == 'I'){
            float val;
            char buf[128];
            c = getchar();
            printf("INPUT COMMAND: \n");
            printf("P - p term\nI - i term\nD - d term\nR - right speed\nL - left speed\n");
            scanf("%c", &c);
            printf("input value: ");
            scanf("%f", &val);
            if(c == 'P'){ sprintf(buf, "P=%f\r\n", val); }
            else if(c == 'I'){ sprintf(buf, "I=%f\r\n", val); }
            else if(c == 'D'){ sprintf(buf, "D=%f\r\n", val); }
            else if(c == 'R'){ sprintf(buf, "SR=%f\r\n", val); }
            else if(c == 'L'){ sprintf(buf, "SL=%f\r\n", val); }
            printf("%s\n", buf);
            write(sock, buf, strlen(buf));
        }
        
        c = getchar();
        recv_from_client(sock, buffer);
        printf("--%s**\n", buffer);
    }
	
	puts("terminating...");
	close(sock);
}

