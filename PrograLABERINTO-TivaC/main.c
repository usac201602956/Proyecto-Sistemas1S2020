/*************************************************** LIBRERIAS DE C ***********************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
/************************************************ LIBRERIAS PARA TIVAC ********************************************************/
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
/**************************************************** DEFINICIONES ************************************************************/
#define ECHO1     GPIO_PIN_0     //PB5
#define ECHO2     GPIO_PIN_1     //PB4
#define ECHO3     GPIO_PIN_4     //PB1
#define ECHO4     GPIO_PIN_5     //PB0
#define ECHOINT1  GPIO_INT_PIN_0
#define ECHOINT2  GPIO_INT_PIN_1
#define ECHOINT3  GPIO_INT_PIN_4
#define ECHOINT4  GPIO_INT_PIN_5
#define TRIGGER1  GPIO_PIN_1     //PE4
#define TRIGGER2  GPIO_PIN_2     //PE3
#define TRIGGER3  GPIO_PIN_3     //PE2
#define TRIGGER4  GPIO_PIN_4     //PE1

#define LEDROJO   GPIO_PIN_1
#define LEDAZUL   GPIO_PIN_2     //PF2
#define LEDVERDE  GPIO_PIN_3     //PF3
#define BOTON     GPIO_PIN_4     //PF4

#define PWMOUT1   GPIO_PIN_0
#define PWMOUT2   GPIO_PIN_1
#define SERVO1    PWM_OUT_0      //PD0
#define SERVO2    PWM_OUT_1      //PD1
/******************************************** CONFIGURACIONES PARA TIVAC ******************************************************/
void TIVACSetup(){
    SysCtlClockSet(SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_2_5); //80MHz.

    //PERIPHERAL Setup
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);   //PUERTO B.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);   //PUERTO D.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);   //PUERTO E.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);   //PUERTO F.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0); //WTIMER0 (32/64bits).
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);  //TIMER0 (16/32bits)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);    //PWM1.

    //GPIO Setup
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ECHO1|ECHO2|ECHO3|ECHO4);                     //ECHOs -------------------------> IN
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, BOTON);
    GPIOPadConfigSet(GPIO_PORTF_BASE, BOTON, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //BOTÓN con Resistencia pull-up

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, TRIGGER1|TRIGGER2|TRIGGER3|TRIGGER4);        //TRIGGERs ----------------------> OUT
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LEDROJO|LEDAZUL|LEDVERDE);                   //LEDROJO, LEDAZUL Y LEDVERDE  --> OUT

    //TIMERS Setup
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_ONE_SHOT);          //WIDE TIMER0 (32/64bits) por disparo.
    TimerLoadSet(WTIMER0_BASE, TIMER_A, 4000000);              //Carga de WTIMER0 para 50ms.
    TimerEnable(WTIMER0_BASE, TIMER_A);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT_UP);        //TIMER0 por disparo ascendente.
    TimerLoadSet(TIMER0_BASE, TIMER_A, 80000000);              //Carga de TIMER0 para 1s

    //INTERRUPT Setup
    IntMasterEnable();
    IntEnable(INT_GPIOB);
    GPIOIntEnable(GPIO_PORTB_BASE, ECHOINT1|ECHOINT2|ECHOINT3|ECHOINT4);
    GPIOIntTypeSet(GPIO_PORTB_BASE, ECHO1|ECHO2|ECHO3|ECHO4, GPIO_BOTH_EDGES); //ECHOs interrumpen por flanco de subida o bajada.

    //PWM Setup
    GPIOPinTypePWM(GPIO_PORTD_BASE, PWMOUT1|PWMOUT2);          //Se establece PD0 y PD1 con PWM.
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);                       //Freq de trabajo PWM1 = 1.25MHz.
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, 25000);              //Señal PWM con T = 20ms --> f = 50Hz.
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);            //Se habilita PWM1 OUT0.
    PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);            //Se habilita PWM1 OUT1.
}
/************************************************ VARIABLES GLOBALES ***********************************************************/
//Para monitorear control PID:
uint32_t LEDdeCarril;
bool     LEDdeCarril_OnOff = false;
bool     start_PID = false;
int      botonStart;
double   periodoDeMuestreo;

//Para sensores ultrasónicos:
uint32_t TRIGGER;
uint32_t ECHO;
bool     estadoDeEcho = false;
bool     tiempoMedido = false;
double   pulseTime;
int      sensor = 1, lastsensor = 1;

//Para datos acerca de la pelotita:
int    posicionPelotita;
double vectorVelocidad[5];
int    numVelocidad = 5;
double T = 0.05;
double sumaVelocidad;

//Para control PID:
double equilibrioServo1 = 1297;  //1297us --> °
double equilibrioServo2 = 1275;  //1283us --> °
double setpoint, error, lasterror=0;
bool   setpointCarril1 = false;
double proporcional, integral, derivativo, sumaPID;
double kp;
double ki;
double kd;
/********************************** SENSORES ULTRASÓNICOS | PROCEDIMIENTOS Y FUNCIONEs *********************************************/
void EchoPulse(){
    ECHO = GPIOIntStatus(GPIO_PORTB_BASE, true);            //Se determina qué ECHO interrumpió.
    GPIOIntClear(GPIO_PORTB_BASE, ECHO);
    pulseTime = 0;
    estadoDeEcho = !estadoDeEcho;                           //Se decide si echo esta en alto o bajo.
    if(estadoDeEcho){
        TimerEnable(TIMER0_BASE, TIMER_A);                  //Inicia conteo de TIMER0.
    }else{
        TimerDisable(TIMER0_BASE, TIMER_A);                 //Se detiene conteo de TIMER0.
        pulseTime = TimerValueGet(TIMER0_BASE, TIMER_A)/80; //Tiempo de pulso en us.
        HWREG(TIMER0_BASE + TIMER_O_TAV) = 0;               //Se resetea TIMER0.
        tiempoMedido = true;
    }
}

double getDistance(int numSensor){
    double distance = 0;
    switch(numSensor){
    case 1:
        TRIGGER = TRIGGER1;
        break;
    case 2:
        TRIGGER = TRIGGER2;
        break;
    case 3:
        TRIGGER = TRIGGER3;
        break;
    case 4:
        TRIGGER = TRIGGER4;
        break;
    default:
        break;
    }
    GPIOPinWrite(GPIO_PORTE_BASE, TRIGGER, TRIGGER);
    SysCtlDelay((0.00001*SysCtlClockGet())/3);       //Delay 10us
    GPIOPinWrite(GPIO_PORTE_BASE, TRIGGER, 0);

    while(!tiempoMedido);
    tiempoMedido = false;

    distance = pulseTime/58;
    return distance;         //Se devuelve la distancia medida en cm.
}
/******************************************** PID | PROCEDIMIENTOS Y FUNCIONES ******************************************************/
/* Procedimiento que permite calcular la velocidad a la que se mueve la pelotita haciendo
 * un promedio de las últimas 5 velocidad leídas.
 */
double promedioVel(){
    int i;
    for(i=0; i<numVelocidad-1; i++){
        vectorVelocidad[i] = vectorVelocidad[i+1];           //Se desplaza los valores una posición hacia la izq. del vector.
    }
    vectorVelocidad[numVelocidad-1] = (error - lasterror)/T; //En la última posición del vector se guarda la última velocidad.
    sumaVelocidad = 0;
    for(i=0; i<numVelocidad; i++){
        sumaVelocidad = sumaVelocidad + vectorVelocidad[i];
    }
    return sumaVelocidad/numVelocidad;                       //Se devuelve la velocidad medida.
}

/* Procedimiento que permite verificar en qué carril del laberinto se encuentra la pelotita.
 * Cuado la encuentra establece el número de sensor ultrasónico que se debe usar según el
 * carril en que se encuentra la pelotita.
 */
int WhereIsTheBall(){
    int numSensor;
    for(numSensor=1; numSensor < 5; numSensor++){
        if(getDistance(numSensor) < 30){
            break;
        }
    }
    return numSensor;  //Se devuelve el numéro de sensor = carril donde esta la pelotita.
}

void Blinking_LEDdeCarril(int numSensor){
    LEDdeCarril_OnOff = !LEDdeCarril_OnOff;  //Se determina si el Led se enciede o apaga.
    switch(numSensor){
    case 1:
        LEDdeCarril = LEDROJO;
        break;
    case 2:
        LEDdeCarril = LEDAZUL;
        break;
    case 3:
        LEDdeCarril = LEDVERDE;
        break;
    case 4:
        LEDdeCarril = LEDAZUL|LEDVERDE;
        break;
    default:
        break;
    }
    if(LEDdeCarril_OnOff){
        GPIOPinWrite(GPIO_PORTF_BASE, LEDROJO|LEDAZUL|LEDVERDE, LEDdeCarril);
    }else{
        GPIOPinWrite(GPIO_PORTF_BASE, LEDROJO|LEDAZUL|LEDVERDE, 0);
    }
}

/* Procedimiento que mueve al Servomotor 1 para que la pelotita complete su
 * recorrido en el último carril.
 */
void MoverServo1(){
    int i;
    GPIOPinWrite(GPIO_PORTF_BASE, LEDROJO|LEDAZUL|LEDVERDE, LEDAZUL|LEDVERDE);
    SysCtlDelay((0.5*SysCtlClockGet())/3);
    for(i=equilibrioServo1; i <= 1412; i++){
        PWMPulseWidthSet(PWM1_BASE, SERVO1, (5*i)/4); //Servo1 se mueve a 80°
        SysCtlDelay((0.008*SysCtlClockGet())/3);
    }
    SysCtlDelay((0.3*SysCtlClockGet())/3);
    for(i=1412; i >= equilibrioServo1; i--){
        PWMPulseWidthSet(PWM1_BASE, SERVO1, (5*i)/4); //Servo1 vuelve al equilibrio.
        SysCtlDelay((0.008*SysCtlClockGet())/3);
    }
    GPIOPinWrite(GPIO_PORTF_BASE, LEDROJO|LEDAZUL|LEDVERDE, 0);

}

/* Procedimiento para mover el Servomotor 2 que se encarga de cambiar a la pelotita
 * de carril cuando el PID le indique hacerlo.
 */
void MoverServo2(){
    int i;
    GPIOPinWrite(GPIO_PORTF_BASE, LEDROJO|LEDAZUL|LEDVERDE, LEDROJO|LEDAZUL|LEDVERDE);
    SysCtlDelay((0.5*SysCtlClockGet())/3);
    for(i=equilibrioServo2; i >= 1188; i--){
        PWMPulseWidthSet(PWM1_BASE, SERVO2, (5*i)/4); //Servo1 se mueve a 80°
        SysCtlDelay((0.008*SysCtlClockGet())/3);
    }
    SysCtlDelay((0.3*SysCtlClockGet())/3);
    for(i=1188; i <= equilibrioServo2; i++){
        PWMPulseWidthSet(PWM1_BASE, SERVO2, (5*i)/4); //Servo1 vuelve al equilibrio.
        SysCtlDelay((0.008*SysCtlClockGet())/3);
    }
}
/****************************************************** CONTROL PID ****************************************************************/
void controlPID(){
    sensor = WhereIsTheBall();
    Blinking_LEDdeCarril(sensor);

    if(sensor == 4){
        MoverServo1();
        sensor = 1;
        lastsensor = 1;
        setpointCarril1 = !setpointCarril1;
        start_PID = false;
    }else{
        posicionPelotita = getDistance(sensor);
        /* Según en qué carril se encuentre la pelotita se determina el
           setpoint a usar. */
        switch(sensor){
        case 1:
            if(setpointCarril1){
                setpoint = 23;
            }else{
                setpoint = 7;
            }
            kp = 2;  //1.5
            ki = 0.5;
            kd = 0;
            break;
        case 2:
            setpoint = 19;
            kp = 1.5;
            ki = 0;
            kd = 0.5;
            break;
        case 3:
            setpoint = 7;
            kp = 2;
            ki = 0.5;
            kd = 0;
            break;
        default:
            break;
        }
        error = setpoint - posicionPelotita;
        proporcional = kp*error;
        derivativo = kd*promedioVel();
        if(abs(error) > 1 ){  //
            integral = 0;
        }else{
            integral += ki*error;
        }
        sumaPID = proporcional + integral + derivativo;
        PWMPulseWidthSet(PWM1_BASE, SERVO1, (5*(equilibrioServo1+sumaPID))/4);
        lasterror = error;
        /* Una vez la pelotita ha llegado cerca de su setpoint el Servomotor 2 la moverá
           al siguiente carril. */
        if(abs(sumaPID) < 2){
            MoverServo2();
        }
    }
}
/******************************************************** MAIN ******************************************************************/
int main(void){
    TIVACSetup();

    while(true){
        botonStart = GPIOPinRead(GPIO_PORTF_BASE, BOTON);
        if(botonStart == 0){
            start_PID = !start_PID;
        }
        if(start_PID){
            periodoDeMuestreo = TimerValueGet(WTIMER0_BASE, TIMER_A);
            if(periodoDeMuestreo == 4000000){
                controlPID();
                TimerEnable(WTIMER0_BASE, TIMER_A);
            }
        }else{
            PWMPulseWidthSet(PWM1_BASE, SERVO1, (5*(equilibrioServo1))/4);
            PWMPulseWidthSet(PWM1_BASE, SERVO2, (5*(equilibrioServo2))/4);
        }
    }
}
