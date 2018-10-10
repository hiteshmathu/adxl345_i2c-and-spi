//  Interfacing ADXL345 accelerometer with MSP430G2553 with I2C communication
//  and printing restuls to serial port using UART.
//
//                                /|\  /|\
//               ADXL345          10k  10k     MSP430G2553
//                slave            |    |        master
//             -----------------   |    |  -----------------
//            |              SDA|<-|---+->|P1.7/UCB0SDA  XIN|-
//            |                 |  |      |                 |
//            |                 |  |      |             XOUT|-
//            |              SCL|<-+----->|P1.6/UCB0SCL     |
//            |                 |         |                 |
//
//  For Sparkfun ADXL345,
//    * connect SDO to ground
//    * connect CS to VCC
//
//  Original Code Made By :
//Mr.Hitesh Mathukiya-known as leonardo de caprio
//Email=hitesh.mathukiya@gmail.com

//******************************************************************************
#include <msp430g2553.h>
#include<math.h>

#define NUM_BYTES_TX 2
#define NUM_BYTES_RX 6
#define ADXL_345     0x53


unsigned char mmm,count=0;
int RXByteCtr, x1,y1,z1;       // enables repeated start when 1
int roll, pitch, yaw,roll_tar=0,pitch_tar=0,yaw_tar=0,tar=0;
volatile unsigned char RxBuffer[6];         // Allocate 6 byte of RAM
unsigned char *PRxData;                     // Pointer to RX data
unsigned char TXByteCtr, RX = 0,minus1=0,minus2=0,minus3=0;
unsigned char MSData[2];

void Setup_TX(unsigned char);
void Setup_RX(unsigned char);
void Transmit(unsigned char,unsigned char);
void TransmitOne(unsigned char);
void Receive(void);
void Setup_UART();
void UARTSendArray(unsigned char *TxArray, unsigned char ArrayLength);
void UARTSendInt(unsigned int x);

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT


    BCSCTL1 = CALBC1_16MHZ; // Set DCO to 1MHz
    DCOCTL = CALDCO_16MHZ; // Set DCO to 1MHz

    // Configure hardware UART
    P1SEL |= BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD

    P2SEL &= ~ BIT2;//pin for switch
    P2SEL2 &= ~ BIT2;
    P2DIR &= ~ BIT2;
    P2REN |= BIT2;
    P2IE |= BIT2;
    P2IES |= BIT2;
    P2IFG &=~ BIT2;


    P2SEL &= ~ BIT0;//pin for CS ALWAYS HIGH
    P2SEL2 &= ~ BIT0;
    P2DIR &= ~ BIT0;
    P2OUT |= BIT0;

    P2SEL &= ~ BIT1;//pin for SDO ALWAYS LOW
    P2SEL2 &= ~ BIT1;
    P2DIR &= ~ BIT1;
    P2OUT &= ~ BIT1;

    // ADXL345
    P1SEL  |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    P1SEL2 |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0

    // Init sequence for ADXL345
    //Transmit process
    Setup_TX(ADXL_345);
    Transmit(0x2D,0x00);                    // STUCK
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

    //Transmit process
    Setup_TX(ADXL_345);
    Transmit(0x2D,0x10);
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

    //Transmit process
    Setup_TX(ADXL_345);
    Transmit(0x2D,0x08);
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

    // Un-comment next block to change range of ADXL345

    Setup_TX(ADXL_345);
    //  RPT_Flag = 1;
    Transmit(0x31,0x03);                            // Range Select at add 0x31 write 0x00 for 2g(default)/ 0x01 for 4g/ 0x02 for 8g/ 0x03 for 16g
    while (UCB0CTL1 & UCTXSTP);         // Ensure stop condition got sent


    long long i;
    __bis_SR_register(GIE );

    while(1){
        Setup_UART();
        UARTSendArray("0\n", 2);

        // loop to measure completion time
        for(i=0;i<1000;i++) {


            if(tar==1){
                roll_tar=roll;
                pitch_tar=pitch;
                yaw_tar=yaw;
                tar=0;

            }

            Setup_TX(ADXL_345);
            TransmitOne(0x32);                                   // Request Data from ADXL345 in 2g Range 10Bit resolution
            while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

            //Receive process
            Setup_RX(ADXL_345);
            Receive();
            while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

            x1 = (((int)RxBuffer[1]) << 8) | RxBuffer[0];
            y1 = (((int)RxBuffer[3]) << 8) | RxBuffer[2];
            z1 = (((int)RxBuffer[5]) << 8) | RxBuffer[4];

            // now You have XYZ axis reading in x1,x2,x3 variable....Bingo... you can play with it as you like....
            // Below if sense x and y angle and Red led is on if its more then 45 or less then -45...
            // you can put your own condition here...

            /* if ((x1 > 128) || (y1 > 128) || (x1 < -128) || (y1 < -128)) { */
            /*   P1OUT |= BIT0; // red led on */
            /* } */
            /* else { */
            /*   P1OUT &= ~BIT0; // red led off */
            /* } */

            /*printf("%d",x1);
              printf("%d",y1);
              printf("%d",z1);*/

            roll = ( ( (atan2(-y1,z1) * 90) / 3.14 ) + 90 );//  Formula for roll
            pitch = ( ( (atan2(z1,x1) * 90) / 3.14 ) + 90 ); // Formula for pitch
            yaw = ( ( (atan2(x1,y1) * 90) / 3.14 ) + 90 ); // Formula for yaw

            /*
            if(x1<0){
                x1= -x1;
                x1=x1*0.3358208955223881;//small
                //x1=x1+90;
                minus1=1;
            }
            else{
                x1=x1*0.3358208955223881;
            }

            if(y1<0){
                y1= -y1;
                y1=y1*0.3515625;//big
                //y1=y1+90;
                minus2=1;
            }
            else{
                y1=y1*0.3515625;
            }

            if(z1<0){
                z1= -z1;
                z1=z1*0.3515625;//flat
                // z1=z1+90;
                minus3=1;
            }
            else{
                z1=z1*0.304054054050541;
            }
             */


            roll=roll-roll_tar;
            pitch=pitch-pitch_tar;
            yaw=yaw-yaw_tar;

            if(roll<0){
                roll= -roll;
            }

            if(pitch<0){
                pitch= -pitch;
            }

            if(yaw<0){
                yaw= -yaw;
            }

            Setup_UART();
            UARTSendArray("R_P_Y\n", 7);
            __delay_cycles(1000000);
            UARTSendInt(roll);
            __delay_cycles(1000000);
            UARTSendInt(pitch);
            __delay_cycles(1000000);
            UARTSendInt(yaw);

            for(mmm=0;mmm<=10;mmm++){
                __delay_cycles(1000000);
            }


            /*UARTSendArray("sample\n", 7);
            if(  minus1==1){
                while(!(IFG2 & UCA0TXIFG)); // Wait for TX buffer to be ready for new data
                UCA0TXBUF =0x2D;  //for minus
                minus1=0;
            }
            // UARTSendInt(x1);
            if(  minus2==1){
                while(!(IFG2 & UCA0TXIFG)); // Wait for TX buffer to be ready for new data
                UCA0TXBUF =0x2D;
                minus2=0;
            }
            // UARTSendInt(y1);
            if(  minus3==1){
                while(!(IFG2 & UCA0TXIFG)); // Wait for TX buffer to be ready for new data
                UCA0TXBUF =0x2D;
                minus3=0;
            }*/
            //  UARTSendInt(z1);
            __delay_cycles(1000000);  // delay 1 sec
            // you can change by changing delay

        }
        Setup_UART();
        //UARTSendArray("Hello\n", 6);
        UARTSendArray("1\n", 2);
        //__delay_cycles(1000);
    }

}

//-------------------------------------------------------------------------------
// The USCI_B0 data ISR is used to move received data from the I2C slave
// to the MSP430 memory. It is structured such that it can be used to receive
// any 2+ number of bytes by pre-loading RXByteCtr with the byte count.
//-------------------------------------------------------------------------------
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{

    if(RX == 1){                              // Master Recieve?
        RXByteCtr--;                              // Decrement RX byte counter
        if (RXByteCtr)
        {
            *PRxData++ = UCB0RXBUF;                 // Move RX data to address PRxData
        }
        else
        {
            UCB0CTL1 |= UCTXSTP;                // No Repeated Start: stop condition
            *PRxData++ = UCB0RXBUF;                   // Move final RX data to PRxData
            __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }}
    else{                                     // Master Transmit
        if (TXByteCtr)                        // Check TX byte counter
        {
            TXByteCtr--;                            // Decrement TX byte counter
            UCB0TXBUF = MSData[TXByteCtr];          // Load TX buffer
        }
        else
        {
            UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
            IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
            __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
    }
}

void Setup_UART() {
    _DINT();
    IE2 &= ~UCB0RXIE;
    IE2 &= ~UCB0TXIE;
    /*UCA0CTL1 |= UCSSEL_2; // Use SMCLK
  UCA0BR0 = 104; // Set baud rate to 9600 with 1MHz clock (Data Sheet 15.3.13)
  UCA0BR1 = 0; // Set baud rate to 9600 with 1MHz clock
  UCA0MCTL = UCBRS0; // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine*/

    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 138;                            // 16MHz 115520
    UCA0BR1 = 0;                              // 16MHz 115520
    UCA0MCTL = UCBRS_7;                         // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    // IE2 &=~ UCA0RXIE;
    IE2 |= UCA0RXIE; // Enable USCI_A0 RX interrupt


}

void Setup_TX(unsigned char Dev_ID){
    _DINT();
    RX = 0;
    IE2 &= ~UCA0RXIE; // Disable USCI_A0 RX interrupt
    IE2 &= ~UCB0RXIE;
    while (UCB0CTL1 & UCTXSTP);               // Ensure stop condition got sent// Disable RX interrupt
    UCB0CTL1 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
    UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
    UCB0BR1 = 0;
    UCB0I2CSA = Dev_ID;                       // Slave Address is 048h
    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
    IE2 |= UCB0TXIE;                          // Enable TX interrupt
}

void Setup_RX(unsigned char Dev_ID){
    _DINT();
    RX = 1;
    IE2 &= ~UCA0RXIE; // Disable USCI_A0 RX interrupt
    IE2 &= ~UCB0TXIE;
    UCB0CTL1 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
    UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
    UCB0BR1 = 0;
    UCB0I2CSA = Dev_ID;                       // Slave Address is 048h
    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
    IE2 |= UCB0RXIE;                          // Enable RX interrupt
}

void Transmit(unsigned char Reg_ADD,unsigned char Reg_DAT){
    MSData[1]= Reg_ADD;
    MSData[0]= Reg_DAT;
    TXByteCtr = NUM_BYTES_TX;                  // Load TX byte counter
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

void TransmitOne(unsigned char Reg_ADD){
    MSData[0]= Reg_ADD;
    TXByteCtr = 1;                  // Load TX byte counter
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

void Receive(void){
    PRxData = (unsigned char *)RxBuffer;    // Start of RX buffer
    RXByteCtr = NUM_BYTES_RX;             // Load RX byte counter
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
    UCB0CTL1 |= UCTXSTT;                    // I2C start condition
    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

void UARTSendArray(unsigned char *TxArray, unsigned char ArrayLength){

    while(ArrayLength--){ // Loop until StringLength == 0 and post decrement
        while(!(IFG2 & UCA0TXIFG)); // Wait for TX buffer to be ready for new data
        UCA0TXBUF = *TxArray; //Write the character at the location specified py the pointer
        TxArray++; //Increment the TxString pointer to point to the next character
    }
    IFG2 &= ~UCA0TXIFG;                     // Clear USCI_A0 int flag
}



void UARTSendInt(unsigned int x){
    unsigned char buff[10];
    unsigned char data[10];
    unsigned char index = 0, i = 0;
    // itoa (x,buff,10);


    while(x > 0) {
        unsigned char val = x % 10;
        if(val < 10)
            buff[index] = 48+val;
        else
            buff[index] = 97+val-10;
        index++;
        x /= 10;
    }
    buff[index] = '\n';

    while(index > 0) {
        index--;
        data[i] = buff[index];
        i++;
    }

    if(i==0) {
        data[0] = '0';
        i++;
    }
    data[i] = '\n';
    UARTSendArray(data, i+1);
}

#pragma vector=PORT2_VECTOR
__interrupt void port_2(void){
    tar=1;
    P2IFG &=~ BIT2;
}
