#include "msp.h"
#include "math.h"
#include "stdio.h"

int keypad_table[4][9] =             //  (Hex) keyx =    0x1,    0x2,    0x4,    0x8
{                                    //  (Bin) keyx = 0b0001, 0b0010, 0b0100, 0b1000
    {0, 68, 70, 0, 48, 0, 0, 0, 69}, // (Char) Row 4:      D,      F,      0,      E
    {0, 67, 57, 0, 56, 0, 0, 0, 55}, // (Char) Row 3:      C,      9,      8,      7
    {0, 66, 54, 0, 53, 0, 0, 0, 52}, // (Char) Row 2:      B,      6,      5,      4
    {0, 65, 51, 0, 50, 0, 0, 0, 49}  // (Char) Row 1:      A,      3,      2,      1
};                                   //             (this is a mirrored keypad table)

int digit_array[16] =
{// (Bin)    =>    (Char)
    0b11000000, // 0
    0b11111001, // 1
    0b10100100, // 2
    0b10110000, // 3
    0b10011001, // 4
    0b10010010, // 5
    0b10000010, // 6
    0b11111000, // 7
    0b10000000, // 8
    0b10010000, // 9
    0b10001000, // A
    0b10000011, // b
    0b11000110, // C
    0b10100001, // d
    0b10000110, // E (*)
    0b10001110  // F (#)
};

void wait(int n)
{
    int i;
    for (i = 0; i < n; i++)
    {
        // do nothing
    }
}

void config_CLK(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    // Power Control Manager (PCM): Set power level for the desired clock frequency
    uint32_t key_bits = 0x695A0000;            // Key to unlock
    uint32_t AM_LDO_VCORE1_bits = 0x00000001;  // AMR Active Mode Request - 01b = AM_LDO_VCORE1
    while (PCM->CTL1 & 0x00000100);            // Wait for PCM to become not busy
    PCM->CTL0 = key_bits | AM_LDO_VCORE1_bits; // Unlock PCM register and set power mode
    while (PCM->CTL1 & 0x00000100);            // Wait for PSM to become not busy
    PCM->CTL0 &= 0x0000FFFF;                   // Lock PCM register again

    // Flash Controller (FLCTL): Flash read wait state number change
    FLCTL->BANK0_RDCTL &=~(BIT(12) | BIT(13) | BIT(14) | BIT(15)); // reset
    FLCTL->BANK0_RDCTL |= (BIT(12));                               // Bit 12 - 15: wait state number selection. 0001b = 1 wait states
    FLCTL->BANK1_RDCTL &=~(BIT(12) | BIT(13) | BIT(14) | BIT(15)); // reset
    FLCTL->BANK1_RDCTL |= (BIT(12));                               // Bit 12 - 15: wait state number selection. 0001b = 1 wait states

    // Clock Source (CS):
    CS->KEY    = (0x695A);                      // KEY     = 695Ah : unlock all CS registers

    // Setup DCOCLK
    CS->CTL0  |= (BIT(23));                     // DCOEN   =    1b : set to enable the clock source
    CS->CTL0  &=~(BIT(18) | BIT(17) | BIT(16)); // DCORSEL =  000b : reset
    CS->CTL0  |= (BIT(17) | BIT(16));           // DCORSEL =  011b : frequency range select = 12MHz (Range 8 - 16)

    // Setup SMCLK (for BRCLK)
    CS->CTL1  &=~(BIT(30) | BIT(29) | BIT(28)); // DIVS    =  000b : SMCLK source divider = f(SMCLK) / 1 = 12MHz
    CS->CTL1  &=~(BIT(6) | BIT(5) | BIT(4));    // SELS    =  000b : reset
    CS->CTL1  |= (BIT(5) | BIT(4));             // SELS    =  011b : selects SMCLK and HSMCLK source = DCOCLK
    CS->CLKEN |= (BIT(3));                      // EN      =    1b : SMCLK system clock conditional request enable

    CS->KEY    = (0x0000);                      // KEY     = 0000h : lock all CS registers

    while(!(CS->STAT & (BIT(27))))              // READY
    {
        // wait for SMCLK to be ready
    }
}

void config_NVIC(void)
{
    NVIC->ISER[0] = (1 << ((EUSCIA2_IRQn) & 0x001F)); // Enable eUSCI_A2
}

void config_GPIO(void)
{
    // Display
    P4->DIR  |= (0xFF); // DIR  = FFh : P4.7-0 output (dp & g-a)
    P4->SEL1 &=~(0xFF); // SEL1 = FFh : P4.7-0 general purpose I/O is selected
    P4->SEL0 &=~(0xFF); // SEL0 = FFh : P4.7-0 general purpose I/O is selected
    P4->OUT  |= (0xFF); // OUT  = FFh :        turn off dp & g-a
    P8->DIR  |= (0x3C); // DIR  = 3Ch : P8.5-2 output (display A-D)
    P8->SEL1 &=~(0x3C); // SEL1 = 3Ch : P8.5-2 general purpose I/O is selected
    P8->SEL0 &=~(0x3C); // SEL0 = 3Ch : P8.5-2 general purpose I/O is selected
    P8->OUT  |= (0x3C); // OUT  = 3Ch :        turn off Display A-D
    // Keys
    P9->DIR  &=~(0x0F); // DIR  = FFh : P9.3-0 input  (keypad row 1-4)
    P9->SEL1 &=~(0x0F); // SEL1 = FFh : P9.3-0 general purpose I/O is selected
    P9->SEL0 &=~(0x0F); // SEL0 = FFh : P9.3-0 general purpose I/O is selected
}

void config_UART(void)
{
    P3->DIR  |= (BIT(3)); // DIR  =  1b : P3.3 output (Tx)
    P3->SEL1 &=~(BIT(3)); // SEL1 =  0b : P3.3 UCA2TXD is selected
    P3->SEL0 |= (BIT(3)); // SEL0 =  1b : P3.3 UCA2TXD is selected

    P3->DIR  &=~(BIT(2)); // DIR  =  0b : P3.2 input (Rx)
    P3->SEL1 &=~(BIT(2)); // SEL1 =  0b : P3.2 UCA2RXD is selected
    P3->SEL0 |= (BIT(2)); // SEL0 =  1b : P3.2 UCA2RXD is selected
}

void config_UCAx(void)
{
    EUSCI_A2->CTLW0 |= (BIT(0));           // SWRST =    1b : software reset enable (eUSCI_A logic) held in reset state

    EUSCI_A2->CTLW0 &=~(BIT(15));          // PAR   =    0b : parity select = disabled
    EUSCI_A2->CTLW0 &=~(BIT(13));          // MSB   =    0b : receive and enter shift register direction = LSB first
    EUSCI_A2->CTLW0 &=~(BIT(12));          // 7BIT  =    0b : character length = 8-bit data
    EUSCI_A2->CTLW0 &=~(BIT(11));          // SPB   =    0b : # of stop bits = one stop bit
    EUSCI_A2->CTLW0 &=~(BIT(10) | BIT(9)); // MODEx =   00b : eUSCI_A mode = UART
    EUSCI_A2->CTLW0 &=~(BIT(8));           // SYNC  =    0b : synchronous mode enable = asychronous

    EUSCI_A2->CTLW0 &=~(BIT(7) | BIT(6));  // SSELx =   00b : reset
    EUSCI_A2->CTLW0 |= (BIT(7));           // SSELx =   10b : eUSCI_A (BRCLK) clock source select = SMCLK = 12MHz

    /*
    1. N = 12MHz / 9600 = 1250.0. Because N > 16, we go to step 3
    2. Skipped
    3. OS16 = 1, UCBRx = int (1250 / 16) = int(78.125) = 78. UCBRFx = int((78.125 - 78) * 16) = 2
    4. UCBRSx = 0x0, because by looking at the table, since N=1250.0, the fractional part of it is 0.
    */

    EUSCI_A2->BRW    = (78);               // BRx   =   78d : clock prescaler setting of the baud-rate generator
    EUSCI_A2->MCTLW &=~(0xFFF0);           // BRSx  = 0000h : BRFx  = 0000h : reset
    EUSCI_A2->MCTLW |= (BIT(5));           // BRFx  = 0020h : first modulation stage select = 2
    EUSCI_A2->MCTLW |= (BIT(0));           // OS16  = 0001h : oversampling mode = enabled

    EUSCI_A2->CTLW0 &=~(BIT(0));           // SWRST =    0b : software reset enable (eUSCI_A logic) released for operation

    EUSCI_A2->IE    &=~(0xF);              // IE    = 0000b : reset
    EUSCI_A2->IE    |= (BIT(1) | BIT(0));  // IE    = 0011b : enable Tx and Rx interrupts
}

#define idle                 (int)(0) // state 0
#define key_press_debounce   (int)(1) // state 1
#define process_keypress     (int)(2) // state 2
#define key_release_debounce (int)(3) // state 3

int debounce_state = idle; // fsm state
int n = 5;                 // bouncing period
int cnt = 0;               // debounce count
int key = 0;               // raw key input
int keyx = 0;              // saved key input
int rowx = 0;              // row #
int ptr = 0;               // Tx_output index
int k = 0;                 // display index

int enter = 0;             // enter key flag
int Tx_output[4] = {};     // transmit array
int Rx_input[4] = {};      // receive array
int Tx_count = 0;          // transmit index
int Rx_count = 3;          // receive index

void display(void)
{
    P8->OUT = ~(0b00000100 << k);       // turn on display[k]
    P4->OUT = 0xFF;                     // turn off dp & g-a
    P4->OUT = digit_array[Rx_input[k]]; // turn on dp & g-a (to display character)
}

void debounce_fsm(void)
{
    wait(500);
    key = P9->IN & (0x0F);                                 // read keypad rows

    switch (debounce_state)
    {
        // State 0: IDLE - FSM stays here until a key is pressed, then logs the input and moves to State 1
        case idle:
            if (key > 0)
            {
                debounce_state = key_press_debounce;
                cnt = 0;
                keyx = key;
                rowx = k;
            }
            else debounce_state = idle;
            break;
        // State 1: KEY PRESS DEBOUNCE - FSM cycles through here until bouncing ceases and then moves to State 2 (LO -> HI)
        case key_press_debounce:
            if (cnt > n) debounce_state = process_keypress;
            else if (k == rowx && key != keyx) debounce_state = idle;
            else if (k == rowx && key == keyx) cnt++;
            break;
        // State 2: PROCESS KEYPRESS - FSM saves the logged input to array to be displayed and then moves to State 3
        case process_keypress:
            if (keypad_table[rowx][keyx] == 68)            // enter key (D) pressed
            {
                enter = 1;
                EUSCI_A2->TXBUF = Tx_output[Tx_count];     // transmit first element
                Tx_count++;
            }
            else                                           // regular key is pressed
            {
                Tx_output[ptr] = keypad_table[rowx][keyx]; // save element
                ptr >= 3 ? ptr = 0 : ptr++;
            }
            cnt = 0;
            debounce_state = key_release_debounce;
            break;
        // State 3: KEY RELEASE DEBOUNCE - FSM cycles through here until bouncing ceases and then returns to State 0 (HI -> LO)
        case key_release_debounce:
            if (cnt > n) debounce_state = idle;
            else if (k == rowx && key != 0) cnt = 0;
            else if (k == rowx && key == 0) cnt++;
            break;
        // Any error will have the FSM reset counters and return to State 0
        default: debounce_state = idle;
    }
}

void EUSCIA2_IRQHandler(void)
{
    uint8_t iv_val = EUSCI_A2->IV;             // read IV register and clear IFG flags
    int temp;

    if ((iv_val & (0x04)) && enter)            // if TXBUF empty and enter key is pressed
    {
        EUSCI_A2->TXBUF = Tx_output[Tx_count]; // transmit character
        Tx_output[Tx_count] = 0;               // clear Tx_output
        Tx_count++;
        if (Tx_count >= 4)                     // reset
        {
            enter = 0;
            Tx_count = 0;
            ptr = 0;
        }
    }
    if (iv_val & (0x02))                       // if RXBUF full
    {
        temp = EUSCI_A2->RXBUF;                // temp for ASCII conversion
        if (temp >= 48 && temp <= 57)          // 0-9
        {
            Rx_input[Rx_count] = temp - 48;    // receive character
            Rx_count--;
        }
        else if (temp >= 65 && temp <= 70)     // A-F
        {
            Rx_input[Rx_count] = temp - 55;    // receive character
            Rx_count--;
        }
        else if (temp >= 97 && temp <= 102)    // a-f
        {
            Rx_input[Rx_count] = temp - 87;    // receive character
            Rx_count--;
        }
        if (Rx_count < 0) Rx_count = 3;        // reset
    }
}

void main(void)
{
    config_CLK();
    config_NVIC();
    config_GPIO();
    config_UART();
    config_UCAx();
    while (1)
    {
        display();
        debounce_fsm();
        k >= 3 ? k = 0 : k++;
    }
}
