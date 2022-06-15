#include "mbed.h"
#include "platform/ATCmdParser.h"
#include "Adafruit_SSD1306.h"
#include "TRSensors.h"
#include "SRF05.h"

#define NUM_SENSORS 5
#define PWMA D6
#define AIN2 A0 // MOTOR - L FORWARD
#define AIN1 A1 // MOTOR - L BACKWARD
#define BIN1 A2 // MOTOR - R FORWARD
#define BIN2 A3 // MOTOR - R BACKWARD
#define PWMB D5
#define MAX 25500
#define INTEGRAL_MAX 1e16

/*****  wifi connection  *****/
BufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 115200);
BufferedSerial wifi(PA_11, PA_12, 115200);
ATCmdParser *parser;


/*****  alphbot contorl variables  *****/
volatile float P_TERM = 0.8;
volatile float I_TERM = 0;
volatile float D_TERM = 0;
volatile float speed_left = 0.392;
volatile float speed_right = 0.4;


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
SRF05 srf05(ARDUINO_UNO_D3, ARDUINO_UNO_D2);


/*****  global variable  *****/
volatile long integral;
volatile float prev_error, my_error, derivative;
// wifi variable
char buffer[128], msg[128], ip[128];
int client_id;
volatile int flag_go = 0;



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
    int _len;
    while(1){
        if(parser->recv("%[^\n]\n", buffer)){
            sscanf(buffer, "+IPD,%d,%d:%[^\n]\n", &client_id, &_len, msg);
            sprintf(msg, "%s\r\n", msg);
            pc.write(msg, strlen(msg));
            print_oled(msg);
            if(strcmp(msg, "go_forward\r\n") == 0){
                pc.write("for\r\n", 5);
                print_oled("for\r\n");
                left_forw(speed_left);
                right_forw(speed_right);
            }
            else if(strcmp(msg, "go_back\r\n") == 0){
                pc.write("bac\r\n", 5);
                print_oled("bac\r\n");
                left_back(speed_left);
                right_back(speed_left);
            }
            else if(strcmp(msg, "go_right\r\n") == 0){
                pc.write("rig\r\n", 5);
                print_oled("rig\r\n");
                left_forw(0.1);
                right_back(0.1);
            }
            else if(strcmp(msg, "go_left\r\n") == 0){
                pc.write("lef\r\n", 5);
                print_oled("lef\r\n");
                left_back(0.1);
                right_forw(0.1);
            }
            else if(strcmp(msg, "go_stop\r\n") == 0){
                pc.write("stp\r\n", 5);
                print_oled("stp\r\n");
                left_back(0);
                right_back(0);
            }
            else if(strcmp(msg, "go_end\r\n") == 0){
                pc.write("end\r\n", 5);
                print_oled("end\r\n");
                return;
            }
            else{
                pc.write("read error\r\n", 12);
                print_oled("read error\r\n");
            }
        }
        else{
            pc.write("error\r\n", 7);
            print_oled("error\r\n");
        }
    }
}

void send_msg(char *_buf){
    int _len;
    parser->send("AT+CIPSEND=%d,%d", client_id, strlen(_buf));
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
    sprintf(buffer, "Waiting for new connection...\r\nip - %s\r\n", ip);
    pc.write(buffer, strlen(buffer));
    print_oled(buffer);
    while(1){
        if(parser->recv("%d,CONNECT", &client_id)){
            pc.write("connection request from a client\r\n", 35);
            print_oled("conenction requets form a client\r\n");
            break;
        }
    }
}

void setup_value(char* msg){
    if (msg[0] == 'R'){
        unsigned long position = trs.readLine(sensorValues);
        sprintf(msg, "read %d %d %d %d %d -> %ld\r\n", sensorValues[0],
                sensorValues[1],sensorValues[2],sensorValues[3],sensorValues[4],position);
    }
    else if (msg[0] == 'C'){
        sprintf(msg, "CALIBRATION DONE\r\n");
        for (int i = 0; i < 100; i ++) 
            trs.calibrate();   
    }
    else if (strstr(msg, "SL=") != NULL) {
        sscanf(msg, "SL=%f", &speed_left);
        sprintf(msg, "SET LEFT SPEED DONE = %.2f\r\n", speed_left);
    }
    else if (strstr(msg, "SR=") != NULL) {
        sscanf(msg, "SR=%f", &speed_right);
        sprintf(msg, "SET RIGHT SPEED DONE = %.2f\r\n", speed_right);
    }
    else if (strstr(msg, "P=") != NULL) {
        sscanf(msg, "P=%f", &P_TERM);
        sprintf(msg, "SET P_TERM DONE= %f\r\n", P_TERM);
    }
    else if (strstr(msg, "I=") != NULL) {
        sscanf(msg, "I=%f", &I_TERM);
        sprintf(msg, "SET I_TERM DONE= %f\r\n", I_TERM);
    }
    else if (strstr(msg, "D=") != NULL) {
        sscanf(msg, "D=%f", &D_TERM);
        sprintf(msg, "SET D_TERM DONE= %f\r\n", D_TERM);
    }
    else if (msg[0] == 'G'){
        sprintf(msg, "GO ALPHABOT2\r\n");
        flag_go = 1;
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
    int _len;
    while(1){
        if(parser->recv("%[^\n]\n", buffer)){
            if(strstr(buffer, "CLOSED") != NULL) {
                sprintf(buffer, "Connection closed\r\nWaiting for new connection...\r\n");
                pc.write(buffer, strlen(buffer));
                print_oled(buffer);
                break;
            }
            else {
                sscanf(buffer, "+IPD,%d,%d:%[^\n]\n", &client_id, &_len, msg);
                sprintf(msg, "%s\r\n", msg);
                pc.write(msg, strlen(msg));
                print_oled(msg);
                setup_value(msg);
                if(flag_go == 1){
                    flag_go = 0;
                    return;
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
    char run_buf[128];
    int position, pid_calculated=0;
    float _left_value, _right_value, normalize = 2000.0;
    float speed_inc = 0;
    integral = derivative = my_error = prev_error = 0;
    while(1){
        position = trs.readLine(sensorValues);
        
        // Range of proportional might be (-2000,2000) 
        // position being 2000 means that the bot is going toward our desired direction
        my_error = (int)position - 2000;
        integral += my_error;
        derivative = prev_error - my_error;
        prev_error = my_error;

        pid_calculated =  (P_TERM * my_error) + (I_TERM * integral) - (derivative * D_TERM);
        if(pid_calculated < 0){ // should turn left
            _left_value = speed_left + pid_calculated / normalize * speed_left;
            _right_value = speed_right;
            left_forw(_left_value);
            right_forw(_right_value);
        } 
        else{ // shoudl turn right
            _left_value = speed_left;
            _right_value = speed_right - pid_calculated / normalize * speed_right;
            left_forw(_left_value);
            right_forw(_right_value);
        }
        sprintf(run_buf, "left = %.2f right = %.2f\r\n", _left_value, _right_value);
        //pc.write(run_buf, strlen(run_buf));
        //send_msg(run_buf);
        print_oled(run_buf);

        if(srf05.read() < 7){
            left_forw(0);
            right_forw(0);
            break;
        }
    }
}

void print_oled(char* _buf){
    oled.clearDisplay();
    oled.setTextCursor(0, 0);
    oled.printf(_buf);
    oled.display();
}

int main(){
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