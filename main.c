//gyro semsor ADXL345
//4 wire spi
//device id 0xE5


#include <msp430.h>
#define SOMI BIT6 //P1.6 Slave out master in
#define SIMO BIT7 //P1.7 Slave in master out
#define SCLK BIT5 //P1.5 Slave Clock
#define CS   BIT4 //P1.5 chip select

void init_clock(void);
void init_gpio_for_gyro(void);
void chip_select_enable_gyro(void);
void chip_select_desable_gyro(void);
void init_spi_for_gyro(void);
void get_device_id_for_gyro(void);
unsigned char device_id_gyro=0,data_formate=0;
unsigned char data_x_L=0,data_x_H=0,data_y_L=0,data_y_H=0,data_z_L=0,data_z_H=0;
void read_data_format_regi(void);
void read_x_y_z_data(void);


int main(void)
{
    init_clock();
    init_gpio_for_gyro();
    init_spi_for_gyro();

    while(1){

        get_device_id_for_gyro();
        read_data_format_regi();
        read_x_y_z_data();
    }


}

void init_clock(void){
    WDTCTL=WDTPW | WDTHOLD;

    if (CALBC1_16MHZ==0xFF)   // If calibration constant erased
    {
        while(1);          // do not load, trap CPU!!
    }

    DCOCTL  = 0;             // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_16MHZ;   // Set range
    DCOCTL  = CALDCO_16MHZ;   // Set DCO step + modulation
}
void init_gpio_for_gyro(void){
    P1SEL = SOMI + SIMO +SCLK  ; // P1.5,6,7 use special function
    P1SEL2 = SOMI + SIMO +SCLK; // P1.5,6,7 use special function
    P1DIR |= CS;   // chip enable
    P1OUT |= CS;
}

void chip_select_enable_gyro(void){
    P1OUT  &= ~ CS;
    // __delay_cycles(200);
}
void chip_select_desable_gyro(void){

    // __delay_cycles(250);

    P1OUT |= CS;

}

void init_spi_for_gyro(void){

    UCB0CTL1 |= UCSWRST;
    UCB0CTL0 &= ~UC7BIT | UCCKPH ;
    UCB0CTL0 = ( UCCKPL | UCMSB | UCMST| UCSYNC ); // Configure the control register of SPI
    UCB0CTL1 |= UCSSEL_2;                           // Configure the control register of SPI

    //Bit rate
    UCB0BR0 = 0X02;
    UCB0BR1 = 00;
    UCB0CTL1 &= ~UCSWRST;

    __delay_cycles(75);


}

void get_device_id_for_gyro(void){
    chip_select_enable_gyro();


    while (!( UCB0TXIFG & IFG2)); //Check if the TXBUF is empty
    UCB0TXBUF = 0x80;//to read data set first bit 1 so here adress is 0x00
    while (!( UCB0TXIFG & IFG2)); // USCI_A0 TX buffer ready?
    UCB0TXBUF = 0xFF;                       // Sending dummy variable to enable the reception
    while(UCB0STAT & UCBUSY);

    device_id_gyro = UCB0RXBUF;  //getting device id put break point here device id is = 0xe5

/*

    while (!( UCB0TXIFG & IFG2)); //Check if the TXBUF is empty

    UCB0TXBUF = 0x31; // We want to write to the data format register while the ADXL345 is in stanby mode


    while (!( UCB0TXIFG & IFG2)); //Check if the TXBUF is empty

    UCB0TXBUF = 0x01; // Write the 0x01 to the data format register (4g range)
*/



    chip_select_desable_gyro();

}


void read_data_format_regi(void){

    chip_select_enable_gyro();

    __delay_cycles(1000);

    while (!( UCB0TXIFG & IFG2)); //Check if the TXBUF is empty
    UCB0TXBUF = 0xB1; //0x31 add  111 0001     11=B
    while (!( UCB0TXIFG & IFG2)); // USCI_A0 TX buffer ready?
    UCB0TXBUF = 0x00;                       // Sending dummy variable to enable the reception
    while(UCB0STAT & UCBUSY);

    data_formate= UCB0RXBUF;

    chip_select_desable_gyro();
    __delay_cycles(1050);

}
void read_x_y_z_data(void){

    chip_select_enable_gyro();

    while (!( UCB0TXIFG & IFG2)); //Check if the TXBUF is empty
    UCB0TXBUF = 0xB2; //0x32
    while (!( UCB0TXIFG & IFG2)); // USCI_A0 TX buffer ready?
    UCB0TXBUF = 0xFF;                       // Sending dummy variable to enable the reception
    while(UCB0STAT & UCBUSY);

    data_x_L= UCB0RXBUF;

    chip_select_desable_gyro();


    chip_select_enable_gyro();

    while (!( UCB0TXIFG & IFG2)); //Check if the TXBUF is empty
    UCB0TXBUF = 0xB3; //0x32
    while (!( UCB0TXIFG & IFG2)); // USCI_A0 TX buffer ready?
    UCB0TXBUF = 0xFF;                       // Sending dummy variable to enable the reception
    while(UCB0STAT & UCBUSY);

    data_x_H= UCB0RXBUF;

    chip_select_desable_gyro();


    chip_select_enable_gyro();

    while (!( UCB0TXIFG & IFG2)); //Check if the TXBUF is empty
    UCB0TXBUF = 0xB4; //0x32
    while (!( UCB0TXIFG & IFG2)); // USCI_A0 TX buffer ready?
    UCB0TXBUF = 0xFF;                       // Sending dummy variable to enable the reception
    while(UCB0STAT & UCBUSY);

    data_y_L= UCB0RXBUF;

    chip_select_desable_gyro();


    chip_select_enable_gyro();

    while (!( UCB0TXIFG & IFG2)); //Check if the TXBUF is empty
    UCB0TXBUF = 0xB5; //0x32
    while (!( UCB0TXIFG & IFG2)); // USCI_A0 TX buffer ready?
    UCB0TXBUF = 0xFF;                       // Sending dummy variable to enable the reception
    while(UCB0STAT & UCBUSY);

    data_y_H= UCB0RXBUF;
    chip_select_desable_gyro();


    chip_select_enable_gyro();
    while (!( UCB0TXIFG & IFG2)); //Check if the TXBUF is empty
    UCB0TXBUF = 0xB6; //0x32
    while (!( UCB0TXIFG & IFG2)); // USCI_A0 TX buffer ready?
    UCB0TXBUF = 0xFF;                       // Sending dummy variable to enable the reception
    while(UCB0STAT & UCBUSY);

    data_z_L= UCB0RXBUF;
    chip_select_desable_gyro();

    chip_select_enable_gyro();
    while (!( UCB0TXIFG & IFG2)); //Check if the TXBUF is empty
    UCB0TXBUF = 0xB7; //0x32
    while (!( UCB0TXIFG & IFG2)); // USCI_A0 TX buffer ready?
    UCB0TXBUF = 0xFF;                       // Sending dummy variable to enable the reception
    while(UCB0STAT & UCBUSY);

    data_z_H= UCB0RXBUF;
    chip_select_desable_gyro();
}
