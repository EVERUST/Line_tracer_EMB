#include "mbed.h"
#include "Adafruit_SSD1306.h"


BufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 115200);

I2C i2c(D14, D15);
Adafruit_SSD1306_I2c oled(i2c, D9, 0x78, 64, 128);


Thread tcp_thread;
ATCmdParser *parser;
BufferedSerial wifi(PA_11, PA_12, 115200);
int _id, _len;
char buffer[128], msg[128];

void tcp(){
    parser = new ATCmdParser(&wifi, "\r\n");
    while(1){
        parser->send("AT+RST");
        if(parser->recv("ready")) {
            sprintf(buffer, "success : AT RST : 3\r\n");
            pc.write(buffer, strlen(buffer));   
        }
        else{
            pc.write("0-fail\r\n", 8);   
            continue; 
        }
        
        parser->send("AT+CWMODE=3");
        if(parser->recv("OK")) {
            sprintf(buffer, "success : set CIPMODE : 3\r\n");
            pc.write(buffer, strlen(buffer));   
        }
        else{
            pc.write("1-fail\r\n", 8);   
            continue; 
        }
        
        parser->send ("AT+CIPAP=\"192.168.0.5\"");
        if (parser->recv("OK")) {
            sprintf(buffer, "success : Set server IP\r\n");
            pc.write(buffer, strlen(buffer));
        }
        else {
            pc.write("2-fail\r\n", 8);   
            continue;   
        }
             
        //parser->send("AT+CWJAP=\"SEUNGYU\",\"21900786\"");
        parser->send("AT+CWSAP=\"esp_yoon\",\"yoon12345\",2,3");
        if(parser->recv("OK")){ 
            sprintf(buffer, "success : set AT+CWSAP\r\n");
            pc.write(buffer, strlen(buffer));
            oled.setTextCursor(0, 0);
            oled.printf("success : set AT+CWSAP\r\n");
            oled.display();
        }
        else{
            pc.write("3-fail\r\n", 8);   
            continue; 
        }
        
        parser->send("AT+CIPMUX=1");
        if (parser->recv("OK")) {
            sprintf(buffer, "success : set CIPMUX : 1\r\n");
            pc.write(buffer, strlen(buffer));
        }
        else{  
            pc.write("4-fail\r\n", 8); 
            continue; 
        }
        
        parser->send("AT+CIPSERVER=1,50000");
        if(parser->recv("OK")) ;
        else{
            pc.write("5-fail\r\n", 8);   
            continue; 
        }
        break;
    }
    oled.printf("Waiting for new connection...\r\n");
    oled.display();
    sprintf(buffer, "Waiting for new connection...\r\n");
    pc.write(buffer, strlen(buffer));
    while(1){
        if(parser->recv("%d,CONNECT", &_id)){
            oled.clearDisplay();
            oled.setTextCursor(0, 0);
            oled.printf("connection request from a client\r\n");
            oled.display();
            pc.write("connection request from a client\r\n", 35);
            while(1){
                if(parser->recv("%[^\n]\n", buffer)){
                    if(strstr(buffer, "CLOSED") != NULL) {
                        sprintf(buffer, "Connection closed\r\nWaiting for new connection...\r\n");
                        pc.write(buffer, strlen(buffer));
                        
                        oled.clearDisplay();
                        oled.setTextCursor(0, 0);
                        oled.printf("connection closed\r\n");
                        oled.display();
                        break;
                    }
                    else {
                        sscanf(buffer, "+IPD,%d,%d:%[^\n]\n", &_id, &_len, msg);
                        sprintf(msg, "%s\r\n", msg);
                        pc.write(msg, strlen(msg));
                        oled.setTextCursor(0, 0);
                        oled.printf("%s\r\n", msg);
                        oled.display();
                        parser->send("AT+CIPSEND=%d,%d", _id, strlen(msg));
                        parser->recv("OK");
                        parser->send("%s", msg);
                        parser->recv("Recv %d bytes", &_len);
                        parser->recv("SEND OK");
                        
                        //TODO run the command 
                        
                    }
                }
            }
        }
    }
    oled.setTextCursor(0, 0);
    oled.printf("done\r\n");
    oled.display();
}

int main(){
    oled.begin(SSD1306_SWITCHCAPVCC);
    oled.clearDisplay();
    oled.setTextCursor(0, 0);
    oled.printf("Start Alphabot2\r\n");
    oled.display();
    
    tcp_thread.start(&tcp);

    while() {
         
    }





    tcp_thread.join();
}
