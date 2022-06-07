#include "mbed.h"
#include "Adafruit_SSD1306.h"


BufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 115200);

I2C i2c(D14, D15);
Adafruit_SSD1306_I2c oled(i2c, D9, 0x78, 64, 128);


Thread tcp_thread;
ATCmdParser *parser;
BufferedSerial wifi(D1, D2, 115200);
int _id, _len;
char buffer[128], msg[128];

/*
void tcp(){
    parser = new ATCmdParser(&wifi);
    parser->set_delimiter("\r\n");

    while(1){
        parser->send("AT+RST");
        if(parser->recv("ready")) ;
        else{ continue; }
                
        parser->send("AT+CWJAP=\"eunchan\",\"eunchanP\"");
        if(parser->recv("OK")){ 
            //pc.write("wifi connected\r\n", 16);
            oled.setTextCursor(0, 0);
            oled.printf("wifi connected\r\n");
            oled.display();
        }
        else{  continue; }
        
        parser->send("AT+CWMODE=3");
        if(parser->recv("OK")) ;
        else{  continue; }

        
        parser->send("AT+CIFSR");
        if(parser->recv("+CIFSR:STAIP,\"%[^\"]\"", buffer) && parser->recv("OK")){
            //pc.write(buffer, strlen(buffer));
            oled.setTextCursor(0, 0);
            oled.printf("%s\r\n", buffer);
            oled.display();
        }
        else{  continue; }
        
        parser->send("AT+CIPMUX=1");
        if(parser->recv("OK")) ;
        else{  continue; }
        
        parser->send("AT+CIPSERVER=1,50000");
        if(parser->recv("OK")) ;
        else{  continue; }
        break;
    }
    //oled.setTextCursor(0, 0);
    oled.printf("Waiting for new connection...\r\n");
    oled.display();
    //sprintf(buffer, "Waiting for new connection...\r\n");
    //pc.write(buffer, strlen(buffer));
    while(1){
        if(parser->recv("%d,CONNECT", &_id)){
            oled.clearDisplay();
            oled.setTextCursor(0, 0);
            oled.printf("connection request from a client\r\n");
            oled.display();
            //pc.write("connection request from a client\r\n", 35);
            while(1){
                if(parser->recv("%[^\n]\n", buffer)){
                    if(strstr(buffer, "CLOSED") != NULL) {
                        //sprintf(buffer, "Connection closed\r\nWaiting for new connection...\r\n");
                        //pc.write(buffer, strlen(buffer));
                        oled.setTextCursor(0, 0);
                        oled.printf("connection closed\r\n");
                        oled.display();
                        break;
                    }
                    else {
                        sscanf(buffer, "+IPD,%d,%d:%[^\n]\n", &_id, &_len, msg);
                        //sprintf(msg, "%s\r\n", msg);
                        //pc.write(msg, strlen(msg));
                        oled.setTextCursor(0, 0);
                        oled.printf("%s\r\n", msg);
                        oled.display();
                        parser->send("AT+CIPSEND=%d,%d", _id, strlen(msg));
                        parser->recv("OK");
                        parser->send("%s", msg);
                        parser->recv("Recv %d bytes", &_len);
                        parser->recv("SEND OK");
                    }
                }
            }
        }
    }
    oled.setTextCursor(0, 0);
    oled.printf("done\r\n");
    oled.display();
    //pc.write("done\r\n", 6);
}
*/
int main(){
    oled.begin(SSD1306_SWITCHCAPVCC);
    
    pc.write("kkthere\r\n", 9);
    oled.clearDisplay();
    pc.write("nowhere\r\n", 9);
    oled.setTextCursor(0, 0);
    oled.printf("hello world\r\n");
    oled.display();
    pc.write("here\r\n", 6);
    

    //tcp_thread.start(&tcp);
       
    
    while(1) ;
    //tcp_thread.join();
}
