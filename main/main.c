/*
 * File:   main.c
 * Author: hsaintandre
 *
 * Created on May 9, 2018, 11:58 AM
 */


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "enc28j60.h"

struct Link {
    char MAC_d[6];  //Destination MAC address
    char MAC_s[6];  //Source MAC address
    char eth_type[2];   //Ethertype by default 0x0800
};
struct IP {
    char ver_ihl;   //default 0x45
    char dscp_ecn;  //default 0x00
    char total_length[2];   //header + tcp + data in bytes
    char id[2]; //default 0x0000
    char flags_n_offset[2]; //flags and offset, set to 0x4000 (don't fragment)
    char ttl;   //default 0x10
    char protocol;  //tcp = 0x06
    char chksum[2]; //to be calculated as sum(header)-chk
    char IP_s[4];   //Source IP
    char IP_d[4];   //Destination IP
};
struct TCP {
    char PORT_s[2]; //Source Port
    char PORT_d[2]; //Destination Port
    char seq_num[4];    //sequence number
    char ack_num[4];    //ACK number
    char offset_n_flags[2]; //set to 0x5003
    char window[2]; //receiver window size
    char chksum[2]; //to be calculated with header IP/TCP and data
    char urg_pointer[2];    //default 0x0000
    char status;
};
struct UDP {
    char PORT_s[2]; //Source Port
    char PORT_d[2]; //Destination Port
    char length[2]; //header and data length
    char flag; //flag for answer
};
struct PING {
    unsigned int checksum;
    char id[2];
    char seq_num[2];
    char payload;
};

static bit next, rxen, arp, ping, init;
char out = 0, arp_type, k=0, opcode[25];
unsigned int repeat = 0, timeout = 0;
unsigned long ack;
char payload[50];
struct PING ping_echo;
struct Link link_tx,link_rx;//link_ping;
struct IP ip_tx,ip_rx;//ip_ping;
struct UDP udp_tx;
struct TCP tcp_tx,tcp_rx;
char ppcontrol = 0;

void init_stack () {
    /* Link layer initialization */
    /*link_tx.MAC_s[0] = 0xB8; link_tx.MAC_s[1] = 0x27; link_tx.MAC_s[2] = 0xEB;
    link_tx.MAC_s[3] = 0xAB; link_tx.MAC_s[4] = 0x52; link_tx.MAC_s[5] = 0x6B;*/
    link_tx.MAC_s[0] = 0x00; link_tx.MAC_s[1] = 0x1E; link_tx.MAC_s[2] = 0xE3;
    link_tx.MAC_s[3] = 0x80; link_tx.MAC_s[4] = 0x0A; link_tx.MAC_s[5] = 0xA4;
    
    /* Internet layer initialization */
    ip_tx.ver_ihl = 0x45;
    ip_tx.dscp_ecn = 0x00;
    ip_tx.total_length[0] = 0 ; ip_tx.total_length[1] = 0;
    ip_tx.id[0] = 0; ip_tx.id[1] = 0;
    ip_tx.flags_n_offset[0] = 0x40; ip_tx.flags_n_offset[1] = 0x00;
    ip_tx.ttl = 0x80;
    ip_tx.protocol = 0x06;
    ip_tx.IP_s[0] = 10; ip_tx.IP_s[1] = 1; ip_tx.IP_s[2] = 109; ip_tx.IP_s[3] = 28;
    
    /* Transport layer initialization */
    udp_tx.PORT_d[0] = 24; udp_tx.PORT_d[1] = 24;
    udp_tx.PORT_s[0] = 24; udp_tx.PORT_s[1] = 24;
    udp_tx.flag = 0;
    
    /* PING initialization */
    ping_echo.checksum = 0x0000;    //pre-calculated chksum
    tcp_tx.PORT_s[0] = 0x09;    //port 2424
    tcp_tx.PORT_s[1] = 0x78;
    tcp_tx.seq_num[0] = 0; tcp_tx.seq_num[1] = 0; tcp_tx.seq_num[2] = 0; tcp_tx.seq_num[3] = 0;
    /*tcp_tx.ack_num[0] = 0; tcp_tx.ack_num[1] = 0; tcp_tx.ack_num[2] = 0; tcp_tx.ack_num[3] = 0;*/
    tcp_tx.offset_n_flags[0] = 0x50;
    tcp_tx.window[0] = 0x00; tcp_tx.window[1] = 25;
    tcp_tx.urg_pointer[0] = 0x00; tcp_tx.urg_pointer[1] = 0x00;
    tcp_tx.status = 0;
}

void uart_init () {
    PIE1bits.RCIE = 1;  //enables EUSART reception interrupt
    PIE1bits.TXIE = 0;  //disables uart transmision interrupt
    TXSTAbits.BRGH = 1;
    BAUDCTLbits.BRG16 = 0;
    //SPBRGH = 0x1A;
    //SPBRG = 0x0A;       //300bps @ Fosc =8MHz
    SPBRGH = 0x00;
    //SPBRG = 0x19;     //9600bps @ Fosc=16MHz BRGH=0
    SPBRG = 0x33;     //9600bps @ Fosc=8MHz BRGH=1
    //SPBRG = 0x0C;       //9600bps @ Fosc=2MHz
    TXSTAbits.SYNC = 0;
    RCSTAbits.SPEN = 1;
    TXSTAbits.TXEN = 1;
    RCSTAbits.CREN = 1; //enables receiver
}

void putch (char data) {      //Mandar esto a otra libreria
    while(!PIR1bits.TXIF)
        continue;
    TXREG = data;
}

void init_timer2 (unsigned char pre, unsigned char post, unsigned char eoc) {
    unsigned char sfr = 0x00;
    PIR1bits.TMR2IF = 0;
    PR2 = eoc;  //sets the end of count
    sfr = ((post-1) << 3) & 0x78;   //places postscaler's bits in their position and applies a mask
    sfr += 4;   //enables timer2 (T2CONbits.TMR2ON = 1;)
    sfr += pre; //places prescaler's bits (00|01|10)
    T2CON = sfr;   
    PIE1bits.TMR2IE = 1;    //enables tmr2 interrupt
}

unsigned char rw_eeprom(unsigned char addr, unsigned char data, unsigned char rw){
    
    if (rw) {   //write = 1
        EEADR = addr;
        EEDAT = data;  //writes data's value on the eeprom data reg
        EECON1bits.EEPGD = 0;   //data memory access
        EECON1bits.WREN = 1;
        EECON2 = 0x55;  //?? mandatory
        EECON2 = 0xAA;  //?? mandatory
        EECON1bits.WR = 1;  //initiates write operation 
        while (!PIR2bits.EEIF); //waits for the end of the write operation
        PIR2bits.EEIF = 0;
        return 0;
    }
    else {      //read = 0
        EEADR = addr;   //sets the address
        EECON1bits.EEPGD = 0;   //data memory access
        EECON1bits.RD = 1;  //initiates read operation
        return EEDAT;  //returns read
    }
}

void init_spi() {
    SSPSTATbits.SMP = 0;
    
    SSPSTATbits.CKE = 1;    //eth
    SSPCONbits.CKP = 0;     //eth
    TRISBbits.TRISB2 = 0;   //CS
    TRISCbits.TRISC3 = 0;   //SCK
    TRISCbits.TRISC4 = 1;   //SDI
    TRISCbits.TRISC5 = 0;   //SDO
    CS = 1;
    //SSPCONbits.SSPM1 = 1;
    //SSPCONbits.SSPM = 0b0010;   //Fsck = Fosc/4
    PIE1bits.SSPIE = 0;     //Interrupt
    SSPCONbits.SSPEN = 1;
}

char spi (unsigned char data, char rw) {
    char dummy, drop;
    
    switch (rw) {
        case 0: //read
            if (SSPCONbits.WCOL) {
                SSPCONbits.WCOL = 0;
                SSPCONbits.SSPEN = 0;
                __delay_ms(1);
                SSPCONbits.SSPEN = 1;
            }
            SSPBUF = data;
            while (!SSPSTATbits.BF);   //every time SSPBSR is loaded and shifted out
            dummy = SSPBUF;     //8 bits are shifted in and transfered to SSPBUF
            break;
        case 1: //write
            if (SSPCONbits.WCOL) {
                SSPCONbits.WCOL = 0;
                SSPCONbits.SSPEN = 0;
                __delay_ms(1);
                SSPCONbits.SSPEN = 1;
            }
            SSPBUF = data;
            while (!SSPSTATbits.BF);   //every time SSPBSR is loaded and shifted out
            drop = SSPBUF;     //8 bits are shifted in and transfered to SSPBUF
            break;
    }
    return dummy;
}

unsigned int join (char *upper, char *lower) {
    unsigned int aux;
    
    aux = *upper;
    aux = aux << 8;
    aux += *lower;
    
    return aux;
}

void config_eth() {
    char ready = 0;
    
    __delay_ms(100);
    RST = 1;
    __delay_ms(100);
    do {
        CS = 0;
        spi(ESTAT | RCR,1);
        ready = spi(0,0) & 0x01;    //checks ESTAT.CLKRDY
        CS = 1;
        __delay_us(100);
    } while (ready == 0);  //waits for initialization  
    /***************************************************   Beginning at Bank 0*/
    CS = 0;
    spi(ECON1 | WCR,1);
    spi(0b00000000,1);    //<7:2> config, <1:0> bank selection
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ECON2 | WCR,1);
    spi(0b10000000,1);    //all in normal mode, address auto increment
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(EIE | WCR,1);
    spi(0b11001011,1);    //global, receive and transmit interrupt enable
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ERXSTL | WCR,1);
    spi(0x00,1);    //Rx start address 0x0000
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ERXSTH | WCR,1);
    spi(0x00,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ERXNDL | WCR,1);
    spi(0xFF,1);    //Rx end address 0x1EFF (4 KB)
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ERXNDH | WCR,1);
    spi(0x1E,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ERDPTL | WCR,1);
    spi(0x00,1);    //read ptr points to start address
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ERDPTH | WCR,1);
    spi(0x00,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ETXSTL | WCR,1);
    spi(0x00,1);    //Tx start address 0x1F00
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ETXSTH | WCR,1);
    spi(0x1F,1);
    CS = 1;
    /*************************************************   Change from Bank 0->1*/
    __delay_us(10);
    CS = 0;
    spi(ECON1 | BFS,1);
    spi(0x01,1);
    CS = 1;
    /**************************************************************************/
    __delay_us(10);
    CS = 0;
    spi(ERXFCON | WCR,1);   //filters
    spi(0b10100000,1);  //broadcast/unicast accepted
    CS = 1;
    /*************************************************   Change from Bank 1->3*/
    __delay_us(10);
    CS = 0;
    spi(ECON1 | BFS,1);
    spi(0x02,1);
    CS = 1;
    /**************************************************************************/
    __delay_us(10);
    CS = 0;
    spi(MAADR6 | WCR,1);
    spi(link_tx.MAC_s[5],1);    //MAC addres from my raspi B8:27:EB:AB:52:6B
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MAADR5 | WCR,1);
    spi(link_tx.MAC_s[4],1);    //MAC addres
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MAADR4 | WCR,1);
    spi(link_tx.MAC_s[3],1);    //MAC addres
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MAADR3 | WCR,1);
    spi(link_tx.MAC_s[2],1);    //MAC addres
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MAADR2 | WCR,1);
    spi(link_tx.MAC_s[1],1);    //MAC addres
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MAADR1 | WCR,1);
    spi(link_tx.MAC_s[0],1);    //MAC addres
    CS = 1;
    /*************************************************   Change from Bank 3->2*/
    __delay_us(10);
    CS = 0;
    spi(ECON1 | BFC,1);
    spi(0x01,1);
    CS = 1;
    /**************************************************************************/
    __delay_us(10);
    CS = 0;
    spi(MACON1 | BFS,1);
    spi(0b00001101,1);    //Enable packets to be received by the MAC
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MACON3 | WCR,1);
    spi(0b10110001,1);    //refer to datasheet (me dio paja) full duplex
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MACON4 | WCR,1);
    spi(0b01000000,1);    //IEEE 802.3 compliance 
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MAMXFLL | WCR,1);
    spi(0xDC,1);    //Max Rx 1500B
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MAMXFLH | WCR,1);
    spi(0x05,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MAIPGL | WCR,1);
    spi(0x12,1);        //who knows
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MAIPGH | WCR,1);
    spi(0x0C,1);        //who knows
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MIREGADR | WCR,1);
    spi(0x00,1);        //PHCON1.PDPXMD matches full/half duplex chose
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MIWRL | WCR,1);
    spi(0x00,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(MIWRH | WCR,1);
    spi(0x01,1);
    CS = 1;
    /*************************************************   Change from Bank 2->1*/
    __delay_us(10);
    CS = 0;
    spi(ECON1 | BFC,1); //2->0
    spi(0x02,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ECON1 | BFS,1); //0->1
    spi(0x01,1);
    CS = 1;
    /**************************************************************************/    
    
    /******************   Rx enable, further modifications are not recommended*/
    __delay_us(10);
    CS = 0;
    spi(ECON1 | BFS,1);
    spi(0b00000100,1);    //ECON1.RXEN bit set, Rx is now enabled
    CS = 1;
    __delay_us(100);
}

unsigned int IP_chksum () {     //some of these fields are constant 
    unsigned long sum = 0;      //a pre-calculation is possible
    unsigned int aux3;
    sum += join(&ip_tx.ver_ihl,&ip_tx.dscp_ecn);
	sum += join(&ip_tx.total_length[0],&ip_tx.total_length[1]);
	sum += join(&ip_tx.id[0],&ip_tx.id[1]);
	sum += join(&ip_tx.ttl,&ip_tx.protocol);
	sum += join(&ip_tx.IP_s[0],&ip_tx.IP_s[1]);
	sum += join(&ip_tx.IP_s[2],&ip_tx.IP_s[3]);
	sum += join(&ip_tx.IP_d[0],&ip_tx.IP_d[1]);
	sum += join(&ip_tx.IP_d[2],&ip_tx.IP_d[3]);
    while (sum > 0x0000FFFF) {
        aux3 = sum >> 16;
        sum = (sum & 0x0000FFFF) + aux3;
    }
    aux3 = sum;
    aux3 = ~aux3;
    
    return aux3;
}

unsigned int TCP_chksum (char cant) {
    unsigned long sum = 0;      //a pre-calculation is possible
    unsigned int aux;
    
    
    sum += join(&ip_tx.IP_s[0],&ip_tx.IP_s[1]);
    sum += join(&ip_tx.IP_s[2],&ip_tx.IP_s[3]);
    sum += join(&ip_tx.IP_d[0],&ip_tx.IP_d[1]);
    sum += join(&ip_tx.IP_d[2],&ip_tx.IP_d[3]);
    sum += ip_tx.protocol;
    sum += cant;
    sum += join(&tcp_tx.PORT_s[0],&tcp_tx.PORT_s[1]);
    sum += join(&tcp_tx.PORT_d[0],&tcp_tx.PORT_d[1]);
    sum += join(&tcp_tx.seq_num[0],&tcp_tx.seq_num[1]);
    sum += join(&tcp_tx.seq_num[2],&tcp_tx.seq_num[3]);
    sum += join(&tcp_tx.ack_num[0],&tcp_tx.ack_num[1]);
    sum += join(&tcp_tx.ack_num[2],&tcp_tx.ack_num[3]);
    sum += join(&tcp_tx.offset_n_flags[0],&tcp_tx.offset_n_flags[1]);
    sum += tcp_tx.window[1];
    cant -= 20;
    if (cant & 0x01) {
        payload[cant] = 0;  //zero padded to even
        cant++;
    }
    cant /= 2;
    for (char i=0;i<cant;i++) {
        aux = payload[2*i];
        aux = aux << 8;
        aux += payload[2*i + 1];
        sum += aux;
    }
    while (sum > 0x0000FFFF) {
        aux = sum >> 16;
        sum = (sum & 0x0000FFFF) + aux;
    }
    aux = sum;
    aux = ~aux;
    
    return aux;
}

void read_buff () {
    char inL, inH, ptrL, ptrH;
    unsigned int aux;
    static bit match;
    
    CS = 0;
    spi(RBM,1);
    ptrL = spi(0x00,0);  //next packet pointer
    ptrH = spi(0x00,0);
    inL = spi(0x00,0);  //packet length
    inH = spi(0x00,0);
    inL = spi(0x00,0);  //status word 2
    inH = spi(0x00,0);
    for (char i=0;i<6;i++) {    //Destination MAC
        link_rx.MAC_d[i] = spi(0,0);
    }
    for (char i=0;i<6;i++) {    //Source MAC
        link_rx.MAC_s[i] = spi(0,0);
    }
    inH = spi(0x00,0);  //E-type
    inL = spi(0x00,0);
    aux = inH;
    aux = (aux << 8) + inL;
    switch (aux) {
        case 0x0800:
            spi(0,0);   //drop fisrt 2B of IPv4 header
            spi(0,0);
            ip_rx.total_length[0] = spi(0,0);   //this should never be greater than 0
            ip_rx.total_length[1] = spi(0,0);
            ip_rx.id[0] = spi(0,0);
            ip_rx.id[1] = spi(0,0);
            for (unsigned int i=0;i<3;i++) {
                spi(0,0);   //drop shit i don't care
            }
            ip_rx.protocol = spi(0,0);
            for (unsigned int i=0;i<2;i++) {
                spi(0,0);   //checksum
            }
            for (char i=0;i<4;i++) {
                ip_rx.IP_s[i] = spi(0,0);   //source IP
            }
            for (char i=0;i<4;i++) {    
                ip_rx.IP_d[i] = spi(0,0);   //destination IP
            }
            for (char i=0;i<4;i++) {
                if (ip_rx.IP_d[i] != ip_tx.IP_s[i]) {   //if it is my IP
                    ip_rx.protocol = 143;   //if this 'if' is executed at least once, then IP doesn't match
                }
            }
            switch (ip_rx.protocol) {
                case 1:     //ICMP                    
                    if (spi(0,0) == 8) {    //echo request
                        ping = 1;
                        ip_tx.protocol = ip_rx.protocol;
                        spi(0,0);   //drop code
                        spi(0,0);   //drop chksum
                        spi(0,0);
                        ping_echo.id[0] = spi(0,0);
                        ping_echo.id[1] = spi(0,0);
                        ping_echo.seq_num[0] = spi(0,0);
                        ping_echo.seq_num[1] = spi(0,0);
                        ping_echo.payload = spi(0,0);
                    }
                    break;
                case 6:     //TCP
                    timeout = 0;    //resets the timeout
                    ack = 0;
                    tcp_tx.PORT_d[0] = spi(0,0);    //client source port
                    tcp_tx.PORT_d[1] = spi(0,0);
                    inH = spi(0,0); //server listen port (2424)
                    inL = spi(0,0); //if port matches, message is accepted
                    if (inH == tcp_tx.PORT_s[0] && inL == tcp_tx.PORT_s[1]) {
                        for (char i=0;i<4;i++) {
                            tcp_rx.seq_num[i] = spi(0,0);  //client seq num is server ack - 1
                            ack = ack << 8;
                            ack += tcp_rx.seq_num[i];
                        }
                        for (char i=0;i<4;i++) {    //ack from sender (my seq+1))
                            //tcp_rx.ack_num[i] = spi(0,0);   
                            tcp_tx.seq_num[i]  = spi(0,0);  //this is cheating, I know
                        }
                        tcp_rx.offset_n_flags[0] = spi(0,0);
                        tcp_rx.offset_n_flags[1] = spi(0,0);
                        for (char i=0;i<6;i++) {
                            spi(0,0);  //drop last 6 bytes
                        }
                        ip_rx.total_length[1] -= 20;    //IP header
                        inL = tcp_rx.offset_n_flags[0] >> 4;    //data offset
                        inL *= 4;
                        ip_rx.total_length[1] -= inL; //TCP header
                        inL -= 20;
                        for (char i=0;i<inL;i++) {
                            spi(0,0);   //drop options
                        }
                        for (char i=0;i<ip_rx.total_length[1];i++) {
                            payload[i] = spi(0,0);  //Data
                        }
                        switch (tcp_tx.status) {
                            case 0:     //listening for a connection, check SYN
                                if (tcp_rx.offset_n_flags[1] & 0x02) {  //if flag SYN is set
                                    tcp_tx.status = 1;
                                }
                                break;
                            case 11:    //SYN sent, waiting for an ACK
                                match = 1;
                                for (char i=0;i<4;i++) {
                                    if (tcp_rx.seq_num[i] != tcp_tx.ack_num[i]) {
                                        match = 0;
                                        tcp_tx.status = 0;
                                    }
                                    if (match) {
                                        tcp_tx.status = 2;
                                    }
                                }
                                break;
                            case 33:    //waiting for data
                                if (tcp_rx.offset_n_flags[1] & 0x01) {  //if flag FIN is set
                                    tcp_tx.status = 5;
                                } else {
                                    tcp_tx.status = 3;  //data
                                }
                                break;
                            case 44:    //data transmitted, waiting an ACK
                                tcp_tx.status = 4;
                                break;
                            case 55:
                                tcp_tx.status = 0;  //FIN ack received
                                break;
                            case 66:
                                if (tcp_rx.offset_n_flags[1] & 0x01) {  //if FIN flag is set
                                    tcp_tx.status = 6;
                                }
                                break;
                        }
                        for (char i=0;i<6;i++) {
                            link_tx.MAC_d[i] = link_rx.MAC_s[i];
                        }
                        for (char i=0;i<4;i++) {
                            ip_tx.IP_d[i] = ip_rx.IP_s[i];
                        }
                    }
                    break;
                case 17:    //UDP
                    udp_tx.PORT_d[0] = spi(0,0);
                    udp_tx.PORT_d[1] = spi(0,0);
                    udp_tx.PORT_s[0] = spi(0,0);
                    udp_tx.PORT_s[1] = spi(0,0);
                    udp_tx.length[0] = spi(0,0);
                    udp_tx.length[1] = spi(0,0) - 8;    //the length must be less than 255
                    spi(0,0);   //drop checksum
                    spi(0,0);
                    for (char i=0;i<udp_tx.length[1];i++) {
                        payload[i] = spi(0,0);
                    }
                    udp_tx.flag = 1;
                    break;
            }
            break;
        case 0x0806:    //ARP
            inH = spi(0,0);
            inL = spi(0,0);
            if (inH == 0 && inL == 1) { //ethernet
                inH = spi(0,0);
                inL = spi(0,0);
                if (inH == 0x08 && inL == 0x00) { //IP
                    inL = spi(0,0);
                    inL = spi(0,0);
                    inL = spi(0,0);
                    arp_type = spi(0,0);
                    if (arp_type > 0) {    //ARP request/reply
                        for (char i=0;i<6;i++) {
                            link_rx.MAC_s[i] = spi(0,0);    //Rx source MAC
                        }
                        for (char i=0;i<4;i++) {
                            ip_rx.IP_s[i] = spi(0,0);    //Rx source IP
                        }
                        for (char i=0;i<6;i++) {
                            link_rx.MAC_d[i] = spi(0,0);    //Rx destination MAC
                        }
                        for (char i=0;i<4;i++) {
                            ip_rx.IP_d[i] = spi(0,0);    //Rx destination IP
                        }
                    }
                }
            }
            break;
    }
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ECON1 | BFC,1); //x->0
    spi(0x03,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ERDPTL | WCR,1);
    spi(ptrL,1);    //next pointer address
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ERDPTH | WCR,1);
    spi(ptrH,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ERXRDPTL | WCR,1);
    spi(ptrL,1);    //next pointer address
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ERXRDPTH | WCR,1);
    spi(ptrH,1);    //next pointer address
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ECON2 | BFS,1);
    spi(0b01000000,1);    //decrement EPKTCNT (must be <255)
    CS = 1;
}

void start_tx () {
    ppcontrol = 0b00000111;
    CS = 0;
    spi(ECON1 | BFC,1); //x->0
    spi(0x03,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(EWRPTL | WCR,1);
    spi(0x00,1);    //next pointer address
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(EWRPTH | WCR,1);
    spi(0x1F,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(WBM,1);
    spi(ppcontrol,1);
}

void close_tx (char aux) {
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ETXNDL | WCR,1);
    spi(aux,1);    //Tx end address
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ETXNDH | WCR,1);
    spi(0x1F,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(EIR | BFC,1);
    spi(0x08,1);    //clear EIR.TXIF
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ECON1 | BFS,1);
    spi(0x08,1);    //enable transmission
    CS = 1;
}

void set_MAC () {
    for (char i=0;i<6;i++) {
        spi(link_tx.MAC_d[i],1);    //
    }
    for (char i=0;i<6;i++) {
        spi(link_tx.MAC_s[i],1);
    }
    for (char i=0;i<2;i++) {    //0x0800 IPv4 packet
        spi(link_tx.eth_type[i],1);
    }
}

void set_IP () {
    spi(ip_tx.ver_ihl,1);    //
    spi(ip_tx.dscp_ecn,1);
    spi(ip_tx.total_length[0],1);    // 
    spi(ip_tx.total_length[1],1);
    spi(ip_tx.id[0],1);   //2B: ID
    spi(ip_tx.id[1],1);
    spi(0,1);   //2B: flags
    spi(0,1);
    spi(ip_tx.ttl,1);
    spi(ip_tx.protocol,1);    //protocol UDP
    spi(ip_tx.chksum[0],1);
    spi(ip_tx.chksum[1],1);
    for (char i=0;i<4;i++) {    //my IP
        spi(ip_tx.IP_s[i],1);
    }
    for (char i=0;i<4;i++) {    //dest IP
        spi(ip_tx.IP_d[i],1);
    }
}

void arp_msg (char direction) { //MAC&IP from both sides must be set
    unsigned int endptr;
    char aux;
    
    ppcontrol = 0b00000111; //automatic padding
    link_tx.eth_type[0] = 0x08;
    link_tx.eth_type[1] = 0x06;   
    CS = 0;
    spi(ECON1 | BFC,1); //x->0
    spi(0x03,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(EWRPTL | WCR,1);
    spi(0x00,1);    //next pointer address
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(EWRPTH | WCR,1);
    spi(0x1F,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(WBM,1);
    spi(ppcontrol,1);
    if (direction == 1) {
        for (char i=0;i<6;i++) {
            spi(0xFF,1);    //MAC_d broadcast
        }
    } else {
        for (char i=0;i<6;i++) {
            spi(link_tx.MAC_d[i],1);
        }
    }
    for (char i=0;i<6;i++) {
        spi(link_tx.MAC_s[i],1);
    }
    for (char i=0;i<2;i++) {    //0x0806 ARP packet
        spi(link_tx.eth_type[i],1);
    }
    spi(0x00,1);    //2B: hardware type = ethernet
    spi(0x01,1);
    spi(0x08,1);    //2B: protocol type = IPv4
    spi(0x00,1);
    spi(0x06,1);    //1B: hardware length
    spi(0x04,1);    //1B: protocol length
    spi(0x00,1);    //2B: request
    spi(direction,1);
    for (char i=0;i<6;i++) {
        spi(link_tx.MAC_s[i],1);
    }
    for (char i=0;i<4;i++) {
        spi(ip_tx.IP_s[i],1);
    }
    for (char i=0;i<6;i++) {
        spi(link_tx.MAC_d[i],1);
    }
    for (char i=0;i<4;i++) {
        spi(ip_tx.IP_d[i],1);
    }
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ETXNDL | WCR,1);
    spi(0x2A,1);    //Tx end address
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ETXNDH | WCR,1);
    spi(0x1F,1);
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(EIR | BFC,1);
    spi(0x08,1);    //clear EIR.TXIF
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ECON1 | BFS,1);
    spi(0x08,1);    //enable transmission
    CS = 1;
}

void echo_ping () { //MAC&IP from both sides must be set
    unsigned int chk;
    char aux2, endptr2;
    unsigned long pchk = 0;
    
    //printf("Echo-ping\n");
    ping = 0;
    /* checksum calculations */
    ping_echo.checksum = 0;
	pchk += join(&ping_echo.id[0],&ping_echo.id[1]);
	pchk += join(&ping_echo.seq_num[0],&ping_echo.seq_num[1]);
	pchk += join(&ping_echo.payload,0);
    while (pchk > 0x0000FFFF) {
        chk = pchk >> 16;
        pchk = (pchk & 0x0000FFFF) + chk;
    }
    ping_echo.checksum = pchk;
    ping_echo.checksum = ~ping_echo.checksum;
    ip_tx.total_length[1] = 29;
    if (ip_tx.id[1] == 255) {
        ip_tx.id[0]++;
    } else {
        ip_tx.id[1]++;
    }
    ip_tx.protocol = 1;
    chk = IP_chksum();
    ip_tx.chksum[1] = chk & 0x00FF;
    ip_tx.chksum[0] = (chk >> 8) & 0x00FF;
    /*************************/
    link_tx.eth_type[0] = 0x08; //IPv4
    link_tx.eth_type[1] = 0x00;
    
    start_tx();
    /******* MAC *******/
    set_MAC();
    /******* IPv4 ******/
    set_IP();
    /******* ICMP ******/
    spi(0,1);   //echo reply
    spi(0,1);   //code = 0
    aux2 = (ping_echo.checksum >> 8) & 0x00FF;
    spi(aux2,1);
    aux2 = ping_echo.checksum & 0x00FF;
    spi(aux2,1);
    spi(ping_echo.id[0],1);
    spi(ping_echo.id[1],1);
    spi(ping_echo.seq_num[0],1);
    spi(ping_echo.seq_num[1],1);
    spi(ping_echo.payload,1);
    close_tx(43);
}

void rx_int (char typeOfRead) { //if typeOfRead is 0, then normal read
    char count;
    if (typeOfRead) {           //if it's 1 due by full fifo, flush memory 
        CS = 0;
        spi(ECON1 | BFC,1);
        spi(0b00000100,1);    //Disable RX in order to free space
        CS = 1;
        __delay_us(10);
        CS = 0;
        spi(EIR | BFC,1);
        spi(0b00000001,1);    //clear RXERIF
        CS = 1;
        __delay_us(10);
        CS = 0;
        spi(ECON1 | BFC,1); //x->0
        spi(0x03,1);
        CS = 1;
        __delay_us(10);
        CS = 0;
        spi(ECON1 | BFS,1); //0->1
        spi(0x01,1);
        CS = 1;
        __delay_us(10);
        CS = 0;
        spi(EPKTCNT,1); //get packet count
        count = spi(0,0);
        CS = 1;
        while (count > 0) { //this reads everything in the buffer
            read_buff();
            __delay_us(10);
            CS = 0;
            spi(EPKTCNT,1); //get packet count
            count = spi(0,0);
            CS = 1;
        }
        __delay_us(10);
        CS = 0;
        spi(ECON1 | BFS,1);
        spi(0b00000100,1);    //ECON1.RXEN bit set, Rx is now enabled
        CS = 1;
    } else {
        CS = 0;
        spi(EIE | BFC, 1);
        spi(0b10000000, 1); //disable global int
        CS = 1;
        __delay_us(10);
        read_buff(); //reads buffer
        __delay_us(10);
        CS = 0;
        spi(EIE | BFS, 1);
        spi(0b10000000, 1); //enable global int
        CS = 1;
    }
}

void tx_int () {
    CS = 0;
    spi(EIR | BFC,1);
    spi(0x08,1);  //clear TXIF
    CS = 1;
    __delay_us(10);
    CS = 0;
    spi(ESTAT, 1);
    if (spi(0,0) & 0x02) { //il faut travailler sur cette merde
        //TX aborted
        //printf("Tx aborted\n");
    }
    CS = 1;
}

void udp_msg(char cant) {   //MAC&IP from both sides must be set and length
    unsigned int aux;
    char arg;
    
    udp_tx.length[1] = 8 + cant;    //udp header + data
    ip_tx.total_length[1] = 20 + udp_tx.length[1];  //ip header + data (udp)
    ip_tx.protocol = 17;    //UDP packet
    aux = IP_chksum();
    ip_tx.chksum[1] = aux & 0x00FF;
    ip_tx.chksum[0] = (aux >> 8) & 0x00FF;

    link_tx.eth_type[0] = 0x08; //IPv4
    link_tx.eth_type[1] = 0x00;
    
    arg = 14 + ip_tx.total_length[1];   //mac header + data (ip + udp)
    
    start_tx();
    /******* MAC *******/
    set_MAC();
    /******* IPv4 ******/
    set_IP();
    /******* UDP ******/
    spi(udp_tx.PORT_s[0],1);
    spi(udp_tx.PORT_s[1],1);
    spi(udp_tx.PORT_d[0],1);
    spi(udp_tx.PORT_d[1],1);
    spi(0,1);   //this belongs to length
    spi(udp_tx.length[1],1);
    spi(0,1);   //2B: checksum IGNORED (weeeee)
    spi(0,1);
    /******* DATA *******/
    for (char i=0;i<cant;i++) {
        spi(payload[i],1);
    }
    /******* END ********/
    close_tx(arg);
}

void tcp_msg(char cant) {   //cant is length of data to Tx
    unsigned long ack_cpy;
    unsigned int aux;
    char arg;
    
    cant += 20;    //tcp header + data
    ip_tx.total_length[1] = 20 + cant;  //ip header + data (tcp)
    ip_tx.protocol = 6;    //TCP packet
    aux = IP_chksum();
    ip_tx.chksum[1] = aux & 0x00FF;
    ip_tx.chksum[0] = (aux >> 8) & 0x00FF;
    
    ack_cpy = ack;
    for (char i=0;i<4;i++) {
        tcp_tx.ack_num[3 - i] = ack_cpy & 0x000000FF;
        ack_cpy = ack_cpy >> 8;
    }
    aux = TCP_chksum(cant);
    tcp_tx.chksum[1] = aux & 0x00FF;
    tcp_tx.chksum[0] = (aux >> 8) & 0x00FF;
    
    
    /************************************/
    link_tx.eth_type[0] = 0x08; //IPv4
    link_tx.eth_type[1] = 0x00;
    
    arg = 14 + ip_tx.total_length[1];   //mac header + data (ip + tcp)
    
    start_tx();
    /******* MAC *******/
    set_MAC();
    /******* IPv4 ******/
    set_IP();
    /******* TCP ********/
    spi(tcp_tx.PORT_s[0],1);
    spi(tcp_tx.PORT_s[1],1);
    spi(tcp_tx.PORT_d[0],1);
    spi(tcp_tx.PORT_d[1],1);
    for (char i=0;i<4;i++) {
        spi(tcp_tx.seq_num[i],1);
    }
    for (char i=0;i<4;i++) {
        spi(tcp_tx.ack_num[i],1);
    }
    spi(tcp_tx.offset_n_flags[0],1);
    spi(tcp_tx.offset_n_flags[1],1);
    spi(tcp_tx.window[0],1);
    spi(tcp_tx.window[1],1);
    spi(tcp_tx.chksum[0],1);
    spi(tcp_tx.chksum[1],1);
    spi(tcp_tx.urg_pointer[0],1);
    spi(tcp_tx.urg_pointer[1],1);
    /******* DATA *******/
    for (char i=0;i<cant;i++) {
        spi(payload[i],1);
    }
    /******* END ********/
    close_tx(arg);
}

void interrupt isr() {
    
    if (PIR1bits.TMR2IF) {
        PIR1bits.TMR2IF = 0;
        char int_vector;

        
        /************ Eth module interrupt service routine ****************/
        if (!DR) {
            CS = 0;
            spi(EIR, 1);
            int_vector = spi(0, 0); //get int vector
            CS = 1;
            __delay_us(10);
            if (int_vector & 0x40) {   //PKTIF set
                rx_int(0);
            }
            if (int_vector & 0x08) {   //TXIF
                tx_int();
            }
            if (int_vector & 0x02) {    //TXERIF
                //printf("TX error\n");
            }
            if (int_vector & 0x01) {    //RXERIF
                //printf("RX error\n");
                rx_int(1);
            }

            if(arp_type == 1) {  //arp replay
                arp_type = 0;
                for (char i=0;i<6;i++) {
                    link_tx.MAC_d[i] = link_rx.MAC_s[i];
                }
                for (char i=0;i<4;i++) {
                    ip_tx.IP_d[i] = ip_rx.IP_s[i];
                }
                arp_msg(2);
            }
            
            if (ping) { //if ping echo request and ARP already solved
                for (char i=0;i<6;i++) {
                    link_tx.MAC_d[i] = link_rx.MAC_s[i]; //copies MAC source to answer
                }
                for (char i=0;i<4;i++) {
                    ip_tx.IP_d[i] = ip_rx.IP_s[i];  //copies IP to answer
                }
                echo_ping();
            }
            
            if (udp_tx.flag) {
                udp_tx.flag = 0;
                putch('U');
                for (char i=0;i<6;i++) {
                    printf("%X:",link_rx.MAC_s[i]);
                }
                for (char i=0;i<4;i++) {
                    printf("%X:",ip_rx.IP_s[i]);
                }
                for (char i=0;i<udp_tx.length[1];i++) {
                    putch(payload[i]);
                }
                putch('\n');
            }
            
            switch (tcp_tx.status) { //handles TCP communication
                case 1: //SYN received, request for connection                    
                    tcp_tx.offset_n_flags[1] = 0x12; //ACK & SYN flags set
                    ack++;
                    tcp_msg(0);
                    tcp_tx.status = 11;
                    break;
                case 2: //ACK received, connection stablished
                    tcp_tx.status = 33;
                    break;
                case 3: //data received, transfer to host and issue an ACK
                    tcp_tx.status = 33;
                    tcp_tx.offset_n_flags[1] = 0x10; //ACK flag set
                    ack += ip_rx.total_length[1];
                    tcp_msg(0); //ack for the received packet
                    putch('T');
                    for (char i=0;i<6;i++) {
                        printf("%X:",link_rx.MAC_s[i]);
                    }
                    for (char i=0;i<4;i++) {
                        printf("%X:",ip_rx.IP_s[i]);
                    }
                    for (char i=0;i<ip_rx.total_length[1];i++) {
                        putch(payload[i]);
                    }
                    putch('\n');
                    /* To be removed */
                    if (payload[0] == 'T') {
                        char adc;
                        
                        for (char i=0;i<6;i++) {
                            link_tx.MAC_d[i] = link_rx.MAC_s[i];
                        }
                        for (char i=0;i<4;i++) {
                            ip_tx.IP_d[i] = ip_rx.IP_s[i];
                        }
                        
                        ADCON0bits.GO = 1;
                        __delay_us(100);
                        while(ADCON0bits.GO);
                        adc = ADRESL;
                        adc /= 3;
                        payload[0] = adc / 10;
                        adc -= (payload[0] * 10);
                        payload[1] = adc;
                        payload[0] += 48;
                        payload[1] += 48;
                        tcp_tx.offset_n_flags[1] = 0x10; //ACK flag set
                        tcp_tx.status = 44;
                        tcp_msg(2);
                    }
                    /*****************/
                    break;
                case 4: //waiting for ACK
                    //if not ack'ed, it should re-transmit
                    tcp_tx.status = 33; //wait for incoming data
                    break;
                case 5: //connection termination received
                    tcp_tx.offset_n_flags[1] = 0x11; //ACK and FIN flags set
                    ack++;
                    tcp_msg(0);
                    tcp_tx.status = 55;
                    break;
                case 6: //issue an ack when FIN from client was received
                    tcp_tx.offset_n_flags[1] = 0x10; //ACK flag set
                    ack++;
                    tcp_msg(0);
                    tcp_tx.status = 0;
                    break;
            }
        }    
        /************ Eth module interrupt service routine ****************/    
        

        if (repeat < 20000) {    //gratuitous ARP
            repeat++;
        } else {    
            repeat = 0;
            if (LED) {
                LED = 0;
            } else {
                LED = 1;
            }
            for (char i=0;i<6;i++) {
                link_tx.MAC_d[i] = 0;
            }
            for (char i=0;i<4;i++) {
                ip_tx.IP_d[i] = ip_tx.IP_s[i];
            }
            arp_msg(1);
            
        }
        if (timeout < 65000) {
            timeout++;
        } else {
            timeout = 0;    //resets the timeout
            if (tcp_tx.status == 33) {  //timeout for tcp connection
                tcp_tx.offset_n_flags[1] = 0x01; //FIN flag set
                tcp_msg(0);
                tcp_tx.status = 66; //waits for ACK
            }
            if (tcp_tx.status == 66) {  //if FIN never received, release resources
                tcp_tx.status = 0;
            }
        }
        
        if (init) { //initialization routine
            if (rw_eeprom(0x00,0x00,0) == 0x0A) {   //this means memory has been initialized
                for (char i=0;i<4;i++) {
                    ip_tx.IP_s[i] = rw_eeprom(i+1,0x00,0);  //get IP
                }
            }
            config_eth();
            init = 0;
        }
    }
    
    if (PIR1bits.RCIF) {
        char drop, j = 0;
        
        while (!PIR1bits.RCIF); //waits for the incoming byte
        if (!RCSTAbits.FERR & !RCSTAbits.OERR) {
            opcode[k] = RCREG;
            k++;
        }  else {
            drop = RCREG;   //reads a char to free up space in the FIFO
            drop = RCREG;   //reads a char to free up space in the FIFO
            RCSTAbits.CREN = 0; //clears OERR
            __delay_us(100);
            RCSTAbits.CREN = 1; //Re-enables receiver
        }
        if (opcode[k-1] == '\n') {  //k stores the total amount of bytes
            switch (opcode[0]) {    //this is the operation code
                case 'I':   //Set source IP address UPPER CASE!
                    j = 0;
                    for (char i=0;i<4;i++) {
                        drop = 0;
                        do {    //this loop starts at cero, multiplies by 10 and adds the number
                            j++;    //thus it can iterate to convert from ascii to integer
                            drop *= 10; //in each iteration, it's multiplied by 10
                            drop += (opcode[j] - 48);   //adds the number
                        } while (opcode[j+1] != '.' && opcode[j+1] != '\n'); //until a dot or LF is found
                        j++;
                        ip_tx.IP_s[i] = drop;
                        rw_eeprom(i+1,drop,1);  //save new IP
                    }
                    rw_eeprom(0x00,0x0A,1); //eeprom initialized 
                    repeat = 20000;  //issue a new ARP with new IP address
                    break;
                case 'i':   //Set destination IP address LOWER CASE!
                    j = 0;
                    repeat = 0; //this avoids the ARP message from overwriting the IP
                    for (char i=0;i<4;i++) {
                        drop = 0;
                        do {    //this loop starts at cero, multiplies by 10 and adds the number
                            j++;    //thus it can iterate to convert from ascii to integer
                            drop *= 10; //in each iteration, it's multiplied by 10
                            drop += (opcode[j] - 48);   //adds the number
                        } while (opcode[j+1] != '.' && opcode[j+1] != '\n'); //until a dot or LF is found
                        j++;
                        ip_tx.IP_d[i] = drop;   
                    }
                    break;
                case 'm':   //Set destination MAC address LOWER CASE!
                    j = 0;
                    repeat = 0; //this avoids the ARP message from overwriting the IP
                    for (char i=0;i<6;i++) {
                        drop = 0;
                        do {    //this loop starts at cero, multiplies by 10 and adds the number
                            j++;    //thus it can iterate to convert from ascii to integer
                            drop *= 16; //in each iteration, it's multiplied by 10
                            if (opcode[j] < 60) {   //these are numbers
                                drop += (opcode[j] - 48);   //adds the number
                            } else {    //these are letters
                                drop += (opcode[j] - 55);
                            }
                        } while (opcode[j+1] != ':' && opcode[j+1] != '\n'); //until a colon or LF is found
                        j++;
                        link_tx.MAC_d[i] = drop; 
                    }
                    break;
                /*case 'P':   //set TCP/UDP port
                    break;
                case 'T':   //request a TCP connection
                    break;*/
                case 'S':   //transmit something (connection required in case of TCP)
                    switch (opcode[1]) {
                        case 'T':   //transmit a message over TCP
                            tcp_tx.offset_n_flags[1] = 0x10; //ACK flag set
                            tcp_tx.status = 44;
                            j = 0;
                            do {
                                payload[j] = opcode[j+2];
                                j++;
                            } while (opcode[j+2] != '\n');
                            tcp_msg(k-3);   //2B opcode, 1B LF
                            break;
                        case 'U':   //transmit a message over UDP (preferred XD)
                            udp_tx.PORT_d[0] = 0x09;
                            udp_tx.PORT_d[1] = 0x78;
                            udp_tx.PORT_s[0] = 0x09;
                            udp_tx.PORT_s[1] = 0x78;
                            j = 0;
                            do {
                                payload[j] = opcode[j+2];
                                j++;
                            } while (opcode[j+2] != '\n');
                            udp_msg(k-3);   //2B opcode, 1B LF
                            break;
                    }
                    break;
            }
            k = 0;
        }
    }
}

void main(void) {
    OSCCON = 0b01110001;    //internal osc @8MHz
    INTCON = 0xC0;  //enables global and peripheral interrupts
    ANSELH = 0x00;
    TRISAbits.TRISA1 = 1;
    ANSEL = 0x02;   //RA1 analog input
    ADCON0bits.ADCS = 0b10;
    ADCON0bits.CHS = 0b001;
    ADCON0bits.ADON = 1;
    ADCON1bits.ADFM = 1;
    //TRISBbits.TRISB5 = 0;   //LED
    TRISCbits.TRISC2 = 0;   //LED
    TRISBbits.TRISB1 = 1;   //DR
    TRISBbits.TRISB0 = 0;   //RST
    LED = 1;
    RST = 0;
    init = 1;
    next = 1;
    rxen = 0;
    ping = 0;
    uart_init();
    init_spi();
    init_stack();
    //config_eth();
    //__delay_ms(4500);
    repeat = 15000;  //starts issuing a gratuitous ARP to update others ARP table
    init_timer2(2, 2, 125); // 2e6/16/2/125 = 500 Hz 
    
    while(1);
}