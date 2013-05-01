#include "msp430g2302.h"
#define TRUE 1
#define FALSE 0
#define BOOLEAN unsigned char
 
///definition des bits des MOTEURS sur P1
#define MotG     (1<<7)//128
#define MotD     (1<<6)//64
///definition des bits des MOTEURS sur P2
#define MotRot   (1<<5)//32
#define MotBras  (1<<4)//16
#define MotPince (1<<3)//8
#define PwmBit   (1<<2)//4 
//#define Torche   (1<<2)//4  //NO PHARE AVAILABLE FOR NOW

///definition des bits des SENS sur P2
#define Sens_2   (1<<1)//2
#define Sens_1   (1<<0)//1

///definition des fins de course sur P1
#define CfcG (1)  //(1<<0)
#define CfcD (8)  //(1<<3)
#define CfcH (16)  //(1<<4)
#define CfcB (32)  //(1<<5)
unsigned char AllCfc= (CfcG + CfcD + CfcH + CfcB );
///definition des actions 
#define BrasHaut    (17)
#define BrasBas     (18)
#define BrasGauche  (33)
#define BrasDroite   (34)
#define Stop  	(0)
//Definition des constantes de temps
#define StopTime   (10)
#define ReverseTime  (100)
 //------------------------------------------------------------------------------
// Hardware-related definitions
//------------------------------------------------------------------------------
#define UART_TXD   0x02                     // TXD on P1.1 (Timer0_A.OUT0)
#define UART_RXD   0x04                     // RXD on P1.2 (Timer0_A.CCI1A)

//------------------------------------------------------------------------------
// Conditions for 9600 Baud SW UART, SMCLK = 1MHz
//------------------------------------------------------------------------------
#define UART_TBIT_DIV_2     (1000000 / (9600 * 2))
#define UART_TBIT           (1000000 / 9600)

//------------------------------------------------------------------------------
// Global variables used for full-duplex UART communication
//------------------------------------------------------------------------------
unsigned int txData;                        // UART internal variable for TX
unsigned char rxBuffer;                     // Received UART character
unsigned char action;                     // Received UART character
///temps d'arret/pause OBLIGATOIRE entre chaque mouvement/action/sollicitation de moteurs
double arret;
unsigned char p1in;
unsigned char cfcs;
BOOLEAN interruption =FALSE;  //interruption available indicator
//------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------
void TimerA_UART_init(void);
void TimerA_UART_tx(unsigned char byte);
void TimerA_UART_print(char *string);
void tempo(double x)
{
  double i,j;
  for(i=0;i < x;i++)
  {
  	for(j=0;j <= 1000/1000;j++)
  	{
  		
  	};	
  };	
}

void makeAction(unsigned char actione)
{
//  unsigned char p2;
  unsigned char action2;
  int t;
  interruption=FALSE;	//initialisation
    ///avant toute action sur un moteur on arette tous les moteurs, puis on fait une temporisation de quelques ms ...
    ///...avant d'enclencher l'action demandee. ce faisant, on s'assure qu'un moteur ne sera jamais sollicité dans un...
    ///sens contraire brutalemnt ce qui pourrait provoquer des appels de courant destructeurs et agir aussi sur la stabilite...
    ///...mecanique du systeme.
    P2OUT=0;
    P1OUT &= ~0xC0;
    tempo(StopTime);
    //Motor Direction
  P2OUT=actione & 0x3F;//(0b00111111);//pas necessaire car P2 n'a pas les bits P2(6) et P2(7)
//p2=P1OUT;
  	///wer save the action value
  	action2=(actione & 0xC0)| ~0xC0 ;
  	t=10;
////    while(!interruption )
    {
  	//we enter a loop to simule a PWM while no interruption is available
////      P1OUT =action2 | PwmBit;	//ON
////      tempo(t);
      P1OUT =action2 & ~PwmBit;	//OFF
////      tempo(t);	
    }
p1in=P1OUT;
}
void makeActionLoop(unsigned char actione)
{
//  unsigned char p2;
  unsigned char action2;
  double t;
  interruption=FALSE;	//initialisation
    ///avant toute action sur un moteur on arette tous les moteurs, puis on fait une temporisation de quelques ms ...
    ///...avant d'enclencher l'action demandee. ce faisant, on s'assure qu'un moteur ne sera jamais sollicité dans un...
    ///sens contraire brutalemnt ce qui pourrait provoquer des appels de courant destructeurs et agir aussi sur la stabilite...
    ///...mecanique du systeme.
    P2OUT=0;
    P1OUT &= ~0xC0;
    tempo(StopTime);
    //Motor + Direction
  action2=actione & 0x3F;//(0b00111111);//pas necessaire car P2 n'a pas les bits P2(6) et P2(7)
//p2=P1OUT;
  	///we save the action value
  	P1OUT=(actione & 0xC0)| ~0xC0 ;
  	t=1;
    while((interruption!=FALSE)&&(rxBuffer!=(unsigned char)(Stop)) )
    {
  	//we enter a loop to simule a PWM while no interruption is available
      P2OUT =action2 | PwmBit;	//PWM T_OFF
      tempo(t*25);//5
  	P2OUT =action2 & ~PwmBit;	//PWM T_ON
      tempo(t);	//10
    }
p1in=P1OUT;
}

// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) 
{
  unsigned char cfcActif;

  ///si un capteur est actionné alors arreter
    if(((( P1IN & CfcH)==0) && action==BrasHaut)//ELEVATION DU BRAS
       ||((( P1IN & CfcB)==0) && action==BrasBas)//ABAISSEMENT DU BRAS
       ||((( P1IN & CfcG)==0) && action==BrasGauche)//ROTATION DU BRAS A GAUCHE
       ||((( P1IN & CfcD)==0) && action==BrasDroite))//ROTATION DU BRAS A DROITE

       {
         p1in=P1IN;
  interruption=TRUE;	//we signal to makeAction function that we passed throw an interruption if it was running before
         	makeAction(Stop);	//1- on arrete tout mouvement pendant qlque ms avant de faire le mouvement contraire sinon DANGER pour les moteurs 
         	///puis en fonction du capteur faire le mouvement contraire pour liberer le capteur + le moteur +son support mecanique
         	if((( P1IN & CfcH)==0))// && action==BrasHaut)//ELEVATION DU BRAS
         	{
         		action=BrasBas;
         		cfcActif=CfcH;
         	}
         	else if(( P1IN & CfcB)==0)
         	{
         		action=BrasHaut;
         		cfcActif=CfcB;
         	}
  		else if(( P1IN & CfcG)==0)
         	{
         		action=BrasDroite;
         		cfcActif=CfcG;
         	}
  		else if(( P1IN & CfcD)==0)
         	{
          	action=BrasGauche;
         		cfcActif=CfcD;
         	}
//  			makeAction(action);	//on fait le mouvement contraire tant que le capteur est encore actif
         	while(( P1IN & cfcActif)==0)
         	{
//  			makeAction(action);	//on fait le mouvement contraire tant que le cqpteur est encore actif
  			//tempo(ReverseTime);	//pendant qlques ms
         	};
  		makeAction(Stop);	//on arrette     
  		action=Stop;		//desactiver les actions avant de sortir en attendant une mise a jour par la ligne SERIAL      		
       }
       P1IFG &= ~AllCfc;         //RAZ des flags des Interruptions pour permettre d'autres interruption sinon bloque les interruptions
}
//------------------------------------------------------------------------------
// main()
//------------------------------------------------------------------------------
void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;               // Stop watchdog timer
    P1SEL = UART_TXD + UART_RXD;            // Timer function for TXD/RXD pins
    P1SEL2 = 0;
    P2SEL = 0x00;
    P2SEL2 = 0x00;

    P1DIR = 0xFF & ~(UART_RXD | AllCfc );               // Set all pins but RXD and Cfcs to output
    P2DIR = 0xFF;

    ///  on eteint rapidement les ports pour eviter etats aleatoire==court circuit possible pour certaines combinaisons des relais de sens!!!!!!
//    P1OUT = 0x00;                           // Initialize all GPIO
//    P2OUT = 0x00;
    
//test
p1in=P1DIR;    
  P1OUT  =  AllCfc ;   	// ON met les pullup a l'etat HAUT
  P2OUT = 0x00;
//p1in=P1OUT ;
  P1REN = AllCfc;  		//Activation des resistances de PullUP pour les capteurs : 1 LoGIQUE au repos et actif au niveau Bas
  P1IE  = AllCfc;          //Activation des Interruptions pour les capteurs
  P1IES = AllCfc;  		//Interruption sur FRONT DESCENDANT pour les capteurs;
  P1IFG &= ~AllCfc;         //RAZ des flags des Interruptions 


    DCOCTL = 0x00;                          // Set DCOCLK to 1MHz
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

//test
cfcs=AllCfc;
    __enable_interrupt();
    
    TimerA_UART_init();                     // Start Timer_A UART
//    TimerA_UART_print("ELEROL3 TimerA UART\r\n");
    TimerA_UART_print("READY.\r\n");
    
    for (;;)
    {
        // Wait for incoming character
        __bis_SR_register(LPM1_bits);
        // Echo received character
        TimerA_UART_tx(rxBuffer);
        action=rxBuffer;  //on memorise l'action
        ///puis on met a jour la derniere valeur demandé par le user en fonction de l'etat des CFCs( Capteur de Fin de Course)...
        ///.. testé dans la routine d'interruption
  	makeActionLoop(action);
        ///...pas necessaire aussi car on a rien branché sur les autres ports P1(0-5)
          							
    }
}
//------------------------------------------------------------------------------
// Function configures Timer_A for full-duplex UART operation
//------------------------------------------------------------------------------
void TimerA_UART_init(void)
{
    TACCTL0 = OUT;                          // Set TXD Idle as Mark = '1'
    TACCTL1 = SCS + CM1 + CAP + CCIE;       // Sync, Neg Edge, Capture, Int
    TACTL = TASSEL_2 + MC_2;                // SMCLK, start in continuous mode
}
//------------------------------------------------------------------------------
// Outputs one byte using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_tx(unsigned char byte)
{
    while (TACCTL0 & CCIE);                 // Ensure last char got TX'd
    TACCR0 = TAR;                           // Current state of TA counter
    TACCR0 += UART_TBIT;                    // One bit time till first bit
    TACCTL0 = OUTMOD0 + CCIE;               // Set TXD on EQU0, Int
    txData = byte;                          // Load global variable
    txData |= 0x100;                        // Add mark stop bit to TXData
    txData <<= 1;                           // Add space start bit
}

//------------------------------------------------------------------------------
// Prints a string over using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_print(char *string)
{
    while (*string) {
        TimerA_UART_tx(*string++);
    }
}
//------------------------------------------------------------------------------
// Timer_A UART - Transmit Interrupt Handler
//------------------------------------------------------------------------------
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
{
    static unsigned char txBitCnt = 10;
//    interruption=TRUE;  //we signal to makeAction function that we passed throw an interruption if it was running before

    TACCR0 += UART_TBIT;                    // Add Offset to CCRx
    if (txBitCnt == 0) {                    // All bits TXed?
        TACCTL0 &= ~CCIE;                   // All bits TXed, disable interrupt
        txBitCnt = 10;                      // Re-load bit counter
    }
    else {
        if (txData & 0x01) {
          TACCTL0 &= ~OUTMOD2;              // TX Mark '1'
        }
        else {
          TACCTL0 |= OUTMOD2;               // TX Space '0'
        }
        txData >>= 1;
        txBitCnt--;
    }
}      
//------------------------------------------------------------------------------
// Timer_A UART - Receive Interrupt Handler
//------------------------------------------------------------------------------
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void)
{ 
    static unsigned char rxBitCnt = 8;
    static unsigned char rxData = 0;

    switch (__even_in_range(TA0IV, TA0IV_TAIFG)) { // Use calculated branching
        case TA0IV_TACCR1:                        // TACCR1 CCIFG - UART RX
            TACCR1 += UART_TBIT;                 // Add Offset to CCRx
            if (TACCTL1 & CAP) {                 // Capture mode = start bit edge
                TACCTL1 &= ~CAP;                 // Switch capture to compare mode
                TACCR1 += UART_TBIT_DIV_2;       // Point CCRx to middle of D0
            }
            else {
                rxData >>= 1;
                if (TACCTL1 & SCCI) {            // Get bit waiting in receive latch
                    rxData |= 0x80;
                }
                rxBitCnt--;
                if (rxBitCnt == 0) {             // All bits RXed?
                    rxBuffer = rxData;           // Store in global variable
                    rxBitCnt = 8;                // Re-load bit counter
interruption=TRUE;  //we signal to makeAction function that we passed throw an interruption if it was running before
                    TACCTL1 |= CAP;              // Switch compare to capture mode
                    __bic_SR_register_on_exit(LPM1_bits);  // Clear LPM0 bits from 0(SR)
                }
            }
            break;
    }
}
//------------------------------------------------------------------------------
