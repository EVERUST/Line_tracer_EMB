#include "mbed.h"
#include "platform/ATCmdParser.h"
#include "Adafruit_SSD1306.h"
#include "TRSensors.h"

#define NUM_SENSORS 5
#define PWMA D6
#define AIN2 A0 // MOTOR - L FORWARD
#define AIN1 A1 // MOTOR - L BACKWARD
#define BIN1 A2 // MOTOR - R FORWARD
#define BIN2 A3 // MOTOR - R BACKWARD
#define PWMB D5
#define MAX 255

/*****  wifi connection  *****/
BufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 115200);
BufferedSerial wifi(PA_11, PA_12, 115200);
ATCmdParser *parser;


/*****  alphbot contorl variables  *****/
volatile int P_TERM = 20;
volatile int I_TERM = 10000;
volatile int D_TERM = 10;
volatile float speed = 1.0;


/*****  alphabot motor control  *****/
// spped
PwmOut left(PWMA);
PwmOut right(PWMB);
// direction
DigitalOut left_fwd(AIN2);
DigitalOut left_bwd(AIN1);
DigitalOut right_fwd(BIN1);
DigitalOut right_bwd(BIN2);


/*****  alphabot OLED  *****/
I2C i2c(D14, D15);
Adafruit_SSD1306_I2c oled(i2c, D9, 0x78, 64, 128);

/*****  alphabot sensors  *****/
TRSensors trs = TRSensors();
unsigned int sensorValues[NUM_SENSORS];



/*****  global variable  *****/
volatile unsigned int last_proportional = 0;
volatile long integral = 0;
volatile int proportional;
volatile int derivative;
volatile bool go = false;
// wifi variable
char buffer[128], msg[128], ip[128];
int _id, _len;



void send_msg(char *_buf);
void setup_server();
void setup_bot();
void setup_value(char* msg);
void manual_control();
void run_bot();
void left_back(float left);
void left_forw(float left);
void right_back(float right);
void right_forw(float right);
void print_oled(char* _buf);

void manual_control(){
    while(1){
        if(parser->recv("%[^\n]\n", buffer)){
            sscanf(buffer, "+IPD,%d,%d:%[^\n]\n", &_id, &_len, msg);
            sprintf(msg, "%s\r\n", msg);
            pc.write(msg, strlen(msg));
            print_oled(msg);
            if(strcmp(msg, "go_forward\r\n") == 0){
                pc.write("for\r\n", 5);
                print_oled("for\r\n");
                left_forw(0.2);
                right_forw(0.2);
            }
            else if(strcmp(msg, "go_back\r\n") == 0){
                pc.write("bac\r\n", 5);
                print_oled("for\r\n");
                left_back(0.2);
                right_back(0.2);
            }
            else if(strcmp(msg, "go_right\r\n") == 0){
                pc.write("rig\r\n", 5);
                print_oled("for\r\n");
                left_forw(0.1);
                right_back(0.1);
            }
            else if(strcmp(msg, "go_left\r\n") == 0){
                pc.write("lef\r\n", 5);
                print_oled("for\r\n");
                left_back(0.1);
                right_forw(0.1);
            }
            else if(strcmp(msg, "go_stop\r\n") == 0){
                pc.write("stp\r\n", 5);
                print_oled("for\r\n");
                left_back(0);
                right_back(0);
            }
            else if(strcmp(msg, "go_end\r\n") == 0){
                pc.write("end\r\n", 5);
                print_oled("for\r\n");
                return;
            }
            else{
                pc.write("read error\r\n", 12);
                print_oled("for\r\n");
            }
        }
        else{
            pc.write("error\r\n", 7);
            print_oled("for\r\n");
        }
    }
}

void send_msg(char *_buf){
    int _len;
    parser->send("AT+CIPSEND=%d,%d", _id, strlen(_buf));
    parser->recv("OK");
    parser->send("%s", _buf);
    parser->recv("Recv %d bytes", &_len);
    parser->recv("SEND OK");
}

void setup_server(){
    pc.write("\r\n\nWiFi TCP Server example\r\n", 28);
    parser = new ATCmdParser(&wifi);
    parser->set_delimiter("\r\n");

    while(1){
        parser->send("AT+RST");
        if(parser->recv("ready")) ;
        else{ 
            pc.write("rst fail\r\n", 10); 
            print_oled("rst fail\r\n"); 
            continue; 
        }
                
        parser->send("AT+CWJAP=\"eunchan\",\"eunchanP\"");
        if(parser->recv("OK")) ;
        else{ 
            pc.write("jap fail\r\n", 10); 
            print_oled("rst fail\r\n"); 
            continue; 
        }
        pc.write("wifi connected\r\n", 16);
        print_oled("wifi connected\r\n"); 
        
        parser->send("AT+CWMODE=3");
        if(parser->recv("OK")) ;
        else{ 
            pc.write("cwm fail\r\n", 10); 
            print_oled("cwm fail\r\n"); 
            continue; 
        }
        pc.write("Set SoftAT+Staion mode\r\n", 24);
        print_oled("Set SoftAT+Staion mode\r\n"); 

        parser->send("AT+CIFSR");
        if(parser->recv("+CIFSR:STAIP,\"%[^\"]\"", ip) && parser->recv("OK")) ;
        else{ 
            pc.write("sip fail\r\n", 10); 
            print_oled("sip fail\r\n"); 
            continue; 
        }
        pc.write(ip, strlen(ip));
        print_oled(ip);
        
        parser->send("AT+CIPMUX=1");
        if(parser->recv("OK")) ;
        else{ 
            pc.write("mux fail\r\n", 10); 
            print_oled("mux fail\r\n"); 
            continue; 
        }
        pc.write("set CIPMUX\r\n", 12);
        print_oled("set CIPMUX\r\n"); 

        parser->send("AT+CIPSERVER=1,50000");
        if(parser->recv("OK")) ;
        else{ 
            pc.write("sev fail\r\n", 10); 
            print_oled("sev fail\r\n"); 
            continue; 
        }
        pc.write("set a TCP server\r\n", 18);
        print_oled("set a TCP server\r\n"); 
        break;
    }
}

void setup_value(char* msg){
    if (msg[0] == 'R'){
        sprintf(msg, "send r back\r\n");
    }
    else if (msg[0] == 'C'){
        sprintf(msg, "CALIBRATION DONE\r\n");
    }
    else if (strstr(msg, "SPEED=") != NULL) {
        sscanf(msg, "SPEED=%f", &speed);
        sprintf(msg, "SET SPEED DONE = %.2f\r\n", speed);
    }
    else if (strstr(msg, "P=") != NULL) {
        sscanf(msg, "P=%d", &P_TERM);
        sprintf(msg, "SET P_TERM DONE= %d\r\n", P_TERM);
    }
    else if (strstr(msg, "I=") != NULL) {
        sscanf(msg, "I=%d", &I_TERM);
        sprintf(msg, "SET I_TERM DONE= %d\r\n", I_TERM);
    }
    else if (strstr(msg, "D=") != NULL) {
        sscanf(msg, "D=%d", &D_TERM);
        sprintf(msg, "SET D_TERM DONE= %d\r\n", D_TERM);
    }
    else if (msg[0] == 'G'){
        sprintf(msg, "GO ALPHABOT2\r\n");
    }
    else if (msg[0] == 'S'){
        sprintf(msg, "STOP ALPHABOT2\r\n");
    }
    else if (strstr(msg, "PID") != NULL) {
        sprintf(msg,"send pid back\r\n");
    }
    else if (msg[0] == 'M'){
        manual_control();
        sprintf(msg, "got M\r\n");
    }
    else sprintf(msg, "syntax err\r\n");
    pc.write(msg, strlen(msg));
    print_oled(msg);
    send_msg(msg);
}

void setup_bot(){
    sprintf(buffer, "Waiting for new connection...\r\nip - %s\r\n", ip);
    pc.write(buffer, strlen(buffer));
    print_oled(buffer);
    while(1){
        if(parser->recv("%d,CONNECT", &_id)){
            pc.write("connection request from a client\r\n", 35);
            print_oled("conenction requets form a client\r\n");
            while(1){
                if(parser->recv("%[^\n]\n", buffer)){
                    if(strstr(buffer, "CLOSED") != NULL) {
                        sprintf(buffer, "Connection closed\r\nWaiting for new connection...\r\n");
                        pc.write(buffer, strlen(buffer));
                        print_oled(buffer);
                        break;
                    }
                    else {
                        sscanf(buffer, "+IPD,%d,%d:%[^\n]\n", &_id, &_len, msg);
                        sprintf(msg, "%s\r\n", msg);
                        pc.write(msg, strlen(msg));
                        print_oled(msg);
                        setup_value(msg);
                    }
                }
            }
        }
    }
}

void left_back(float _left){
    left_fwd = 0;
    left_bwd = 1;
    left = _left;
}
void left_forw(float _left){
    left_fwd = 1;
    left_bwd = 0;
    left = _left;
}
void right_back(float _right){
    right_fwd = 1;
    right_bwd = 0;
    right = _right;
}
void right_forw(float _right){
    right_fwd = 0;
    right_bwd = 1;
    right = _right;
}

void run_bot(){
    while(1){
        unsigned int position = trs.readLine(sensorValues);
        
        //Range of proportional might be (-2000,2000) 
        proportional = (int)position - 2000;
        derivative = proportional - last_proportional;
        integral += proportional;
        
        last_proportional = proportional;
        float power_difference = proportional/P_TERM + integral/I_TERM + derivative*D_TERM;
        
        //int sum = power_difference + MAX
        if (power_difference > MAX)
            power_difference = MAX;
        if (power_difference < (MAX) * (-1))
            power_difference = (MAX) * (-1);
        
        float left_value, right_value;
        if (power_difference < 0) {
            left_value = (float) (MAX + power_difference) / MAX;
            right_value = (float) MAX / MAX;
            left = left_value * speed;
            right = right_value * speed;
        } 
        else {
            left_value = (float) MAX / MAX;
            right_value = (float) (MAX - power_difference) / MAX;
            left = left_value * speed;
            right = right_value * speed;
        }   
        
        if(sensorValues[1] > 900 && sensorValues[2] > 900 && sensorValues[3] > 900) {
            left_value = 0;
            right_value = 0;
            left = 0;
            right = 0;   
        }
        
        oled.setTextCursor(0, 0);
        oled.printf("left = %.2f \r\nright = %.2f\r\n", left_value, right_value);
        oled.display();
    }
}

void print_oled(char* _buf){
    oled.clearDisplay();
    oled.setTextCursor(0, 0);
    oled.printf(_buf);
    oled.display();
}

int main()
    oled.begin(SSD1306_SWITCHCAPVCC);
    oled.clearDisplay();
    oled.setTextCursor(0, 0);
    oled.printf("Start Alphabot2\r\n");
    oled.display();
    setup_server();
    while(1){
        setup_bot();
        run_bot();
    }
    
    pc.write("done\r\n", 6);
    while(1) {} 
}
