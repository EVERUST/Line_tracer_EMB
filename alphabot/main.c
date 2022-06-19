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

/*****  wifi connection  *****/
BufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 115200);
BufferedSerial wifi(PA_11, PA_12, 115200);
ATCmdParser *parser;


/*****  alphbot contorl variables  *****/
volatile float P_TERM = 0.45; //0.5
volatile float I_TERM = 0.04; //0.01
volatile float D_TERM = 0.5; //1
volatile long INTEGRAL_MAX = 100000;
volatile float speed_left = 0.4225; //0.65;//0.65
volatile float speed_right = 0.429; //0.655;//0.655;


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
volatile float prev_error, curr_error, derivative;
// wifi variable
char buffer[128], ip[128];
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
    char _msg[256];
    while(1){
        if(parser->recv("%[^\n]\n", buffer)){
            sscanf(buffer, "+IPD,%d,%d:%[^\n]\n", &client_id, &_len, _msg);
            sprintf(_msg, "%s\r\n", _msg);
            pc.write(_msg, strlen(_msg));
            print_oled(_msg);
            if(strcmp(_msg, "go_forward\r\n") == 0){
                pc.write("for\r\n", 5);
                print_oled("for\r\n");
                left_forw(speed_left * speed_left);
                right_forw(speed_right * speed_right);
            }
            else if(strcmp(_msg, "go_back\r\n") == 0){
                pc.write("bac\r\n", 5);
                print_oled("bac\r\n");
                left_back(speed_left * speed_left);
                right_back(speed_right * speed_right);
            }
            else if(strcmp(_msg, "go_right\r\n") == 0){
                pc.write("rig\r\n", 5);
                print_oled("rig\r\n");
                left_forw(0.1);
                right_back(0.1);
            }
            else if(strcmp(_msg, "go_left\r\n") == 0){
                pc.write("lef\r\n", 5);
                print_oled("lef\r\n");
                left_back(0.1);
                right_forw(0.1);
            }
            else if(strcmp(_msg, "go_stop\r\n") == 0){
                pc.write("stp\r\n", 5);
                print_oled("stp\r\n");
                left_back(0);
                right_back(0);
            }
            else if(strcmp(_msg, "go_end\r\n") == 0){
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

void setup_value(char* _msg){
    if (_msg[0] == 'R'){
        unsigned long position = trs.readLine(sensorValues);
        sprintf(_msg, "read %d %d %d %d %d -> %ld\r\n", sensorValues[0],
                sensorValues[1],sensorValues[2],sensorValues[3],sensorValues[4],position);
    }
    else if (_msg[0] == 'C'){
        sprintf(_msg, "CALIBRATION DONE\r\n");
        for (int i = 0; i < 100; i ++) 
            trs.calibrate();   
    }
    else if (strstr(_msg, "SL=") != NULL) {
        sscanf(_msg, "SL=%f", &speed_left);
        sprintf(_msg, "SET LEFT SPEED DONE = %.2f\r\n", speed_left);
    }
    else if (strstr(_msg, "SR=") != NULL) {
        sscanf(_msg, "SR=%f", &speed_right);
        sprintf(_msg, "SET RIGHT SPEED DONE = %.2f\r\n", speed_right);
    }
    else if (strstr(_msg, "P=") != NULL) {
        sscanf(_msg, "P=%f", &P_TERM);
        sprintf(_msg, "SET P_TERM DONE= %f\r\n", P_TERM);
    }
    else if (strstr(_msg, "I=") != NULL) {
        sscanf(_msg, "I=%f", &I_TERM);
        sprintf(_msg, "SET I_TERM DONE= %f\r\n", I_TERM);
    }
    else if (strstr(_msg, "D=") != NULL) {
        sscanf(_msg, "D=%f", &D_TERM);
        sprintf(_msg, "SET D_TERM DONE= %f\r\n", D_TERM);
    }
    else if (_msg[0] == 'G'){
        sprintf(_msg, "GO ALPHABOT2\r\n");
        flag_go = 1;
    }
    else if (strstr(_msg, "PID") != NULL) {
        sprintf(_msg,"send pid back\r\n");
    }
    else if (_msg[0] == 'M'){
        manual_control();
        sprintf(_msg, "got M\r\n");
    }
    else sprintf(_msg, "syntax err\r\n");
    pc.write(_msg, strlen(_msg));
    print_oled(_msg);
    send_msg(_msg);
}

void setup_bot(){
    int _len;
    char _msg[256];
    while(1){
        if(parser->recv("%[^\n]\n", buffer)){
            if(strstr(buffer, "CLOSED") != NULL) {
                sprintf(buffer, "Connection closed\r\nWaiting for new connection...\r\n");
                pc.write(buffer, strlen(buffer));
                print_oled(buffer);
                break;
            }
            else {
                sscanf(buffer, "+IPD,%d,%d:%[^\n]\n", &client_id, &_len, _msg);
                sprintf(_msg, "%s\r\n", _msg);
                pc.write(_msg, strlen(_msg));
                print_oled(_msg);
                setup_value(_msg);
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

inline float square(float x){
    return x * x;
}
void run_bot(){
    char run_buf[128];
    float position, pid_calculated=0;
    float _left_value, _right_value, normalize = 2000.0;
    integral = derivative = curr_error = prev_error = 0;
    int p_flag = 0;
    while(1){
        position = trs.readLine(sensorValues);
        
        // Range of proportional might be (-2000,2000) 
        // position being 2000 means that the bot is going toward our desired direction
        curr_error = position - 2000.0;
        integral += curr_error;
        derivative = prev_error - curr_error;
        prev_error = curr_error;

        if((integral < 0 && curr_error > 0) || (integral > 0 && curr_error < 0)){
            integral /= 2;
            if(integral < curr_error * -1)
                integral = 0;
        }
        if(integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
        p_flag = 0;
        if(curr_error > 1500 || curr_error < -1500) p_flag = 1;
        pid_calculated =  (p_flag * P_TERM * curr_error) + (I_TERM * integral) - (derivative * D_TERM);

        if(pid_calculated < 0){ // should turn left
            _left_value = 1 + pid_calculated / normalize;
            _right_value = 1;
            left_forw(square(_left_value) * _left_value * speed_left);
            right_forw(speed_right);
        } 
        else{ // shoudl turn right
            _left_value = 1;
            _right_value = 1 - pid_calculated / normalize;
            left_forw(speed_left);
            right_forw(square(_right_value) * _right_value * speed_right);
        }
        sprintf(run_buf, "left = %.2f right = %.2f\r\n", _left_value, _right_value);
        //pc.write(run_buf, strlen(run_buf));
        //send_msg(run_buf);
        print_oled(run_buf);
        
        
        if(srf05.read() < 10){
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