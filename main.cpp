/*
* Author: Tijmen Van Der Beek
* Purpose: To make an automatic trash can.
* Assignment: Project 3
* Extra Subroutines:
* Functions
*   init();                                     initilizes the program
*
* Interrupt Handlers
*   lid_rise_ir_handler();                      this handles the fall section of the lid ir sensor
*   lid_fall_ir_handler();                      this handles the rise section of the lid ir sensor
*   trash_ir_handler();                         ir sensor for handling the trash light
*   button_handler();                           this handles the on board push button
*
* Drivers
*   initLid(leftServo, rightServo);             initilizes the lid of the trash can
*   openLid(leftServo, rightServo, time, res);  opens the lid of the trash can
*   closeLid(leftServo, rightServo, time, res); closes the lid of the trash can
*   setPin(Port, Pin, Mode);                    initilizes a gpio pin
*   readPin(Port, Pin);                         reads a gpio pin
*   writePin(Port, Pin, state);                 writes to a gpio pin
*   flipPin(Port, Pin);                         flips the bit of a gpio pin
*   pullPin(Port, Pin, Mode);                   sets the pull mode of a gpio pin
* ----------------------------------------------------
* Inputs: front IR sensor
*         inside IR sensor
*         onboard Push button
* Outputs: LED
*          Left Solenoid
*          Right Solenoid
* Constraints: When front IR sensor activates the solenoids activates
*              When the inside IR sensor activates the LED activates
*              When the onboard bush button is pressed switch modes
* ----------------------------------------------------
* Refernces: M0432 Reference Manual
*            UM2179 User Manual
*            armMBED Website Documentaion
*/
//-------------------Librarys-------------------//
#include "mbed.h"


//-------------------Definitions-------------------//
//modes for setPin
#define READ 0
#define WRITE 1
#define ALT_FUNC 2
#define ANALOG 3

//modes for pullPin
#define NO_PULL 0
#define PULL_UP 1
#define PULL_DOWN 2

//timeout time for watchdog timer (ms)
#define TIMEOUT 3000

//mode for trash can
#define WATCHDOG_MODE 0
#define IR_MODE 1


//-------------------Functions-------------------//
//Extra Functions
void init(); //initilizes the program

//Interrupt Handlers
void lid_rise_ir_handler(); //this handles the fall section of the lid ir sensor
void lid_fall_ir_handler(); //this handles the rise section of the lid ir sensor
void trash_ir_handler(); //ir sensor for handling the trash light
void button_handler(); //this handles the on board push button

//Drivers
void initLid(PwmOut* leftServo, PwmOut* rightServo); //initilizes the lid of the trash can
void openLid(PwmOut* leftServo, PwmOut* rightServo, uint32_t time, uint32_t res); //opens the lid of the trash can
void closeLid(PwmOut* leftServo, PwmOut* rightServo, uint32_t time, int res); //closes the lid of the trash can
void setPin(GPIO_TypeDef* Port, uint16_t Pin, uint8_t Mode); //initilizes a gpio pin
bool readPin(GPIO_TypeDef* Port, uint16_t Pin); //reads a gpio pin
void writePin(GPIO_TypeDef* Port, uint16_t Pin, bool state); //writes to a gpio pin
void flipPin(GPIO_TypeDef* Port, uint16_t Pin); //flips the bit of a gpio pin
void pullPin(GPIO_TypeDef* Port, uint16_t Pin, uint8_t Mode); //sets the pull mode of a gpio pin


//-------------------Variables-------------------//
//Servo Signal
PwmOut servo_r(PE_9); //right servo
PwmOut servo_l(PE_11); //left servo

//Interrupts
InterruptIn lid_ir(PA_3); //ir sensor for opening lid
InterruptIn trash_ir(PC_0); //ir sensor for detecting if trash is full
InterruptIn mode_button(PC_13); //on_board push buttton to change mode of trash can

//Threads
Thread t;

//EventQueues
EventQueue queue(32 * EVENTS_EVENT_SIZE);

//Watchdog
Watchdog &wdTimer = Watchdog::get_instance();

//Variables
bool is_lid_open = false; //this is if the lid is open or not
bool opening_lid_lock = false;  //this is a lock for while the lid is opening
bool closing_lid_lock = false; //this is a lock for while the lid is closing
bool rise_lock = false; //this is a lock determining if the lid_rise_handler is in the queue
bool fall_lock = true;  //this is a lock determining if the lid_fall_handler is in the queue
bool mode = WATCHDOG_MODE; //This is the mode the trash can is in

//besides initilization everything is done in the interrupts and EventQueue
int main()
{
    //initilizes program
    init();

    //runs forever
    while(true){}
}


//-------------------Extra Functions-------------------//
/*
*/
void init(){
    //initilizes the servos for the lid and sets them to the closed position 
    initLid(&servo_l, &servo_r);
    
    //initilizes the ir sensors and led pins on the board
    setPin(GPIOA, 3, READ); //ir_sensor lid
    setPin(GPIOC, 0, READ); //ir_sensor full trash
    setPin(GPIOC, 3, WRITE); //led

    //sets interrupt pins into pulldown mode
    pullPin(GPIOA, 3, PULL_DOWN);
    pullPin(GPIOC, 0, PULL_DOWN);

    //sets up interrupts for the ir sensors
    lid_ir.rise(&lid_rise_ir_handler);
    lid_ir.fall(&lid_fall_ir_handler);
    trash_ir.rise(&trash_ir_handler);
    trash_ir.fall(&trash_ir_handler);
    
    //initilizing the on board push button to read mode and setting the intrrupt for it 
    setPin(GPIOC, 13, 0);
    mode_button.rise(&button_handler);

    //start the EventQueue queue in thread t
    t.start(callback(&queue, &EventQueue::dispatch_forever));

    //enables the interrupts for the ir sensors
    lid_ir.enable_irq();
    trash_ir.enable_irq();
}


//-------------------Interrupt Handlers-------------------//
/*
    This handles the rise interrupt for the ir sensor in the front of the trash can
    There are two modes:
    WATCHDOG_MODE
        It opens the lid once then keeps kicking the watchdog timer until the fall handler

    IR_MODE
        If the IR sensor is activated it should open the lid 
        If the lid is in the middle of closing the rise handler should be added to the queue so that it runs after the lid closes
    
    (This IR sensor gives a 0 when there is notheing detected and a 1 when something is)
*/
void lid_rise_ir_handler(){
    //WATCHDOG_MODE
    if(mode == WATCHDOG_MODE){  
        //if lid is not open or opening add openLid to the queue
        if(!is_lid_open && !opening_lid_lock){
            queue.call(openLid, &servo_l, &servo_r, 0, 1);
            
            opening_lid_lock = true;    //sets lid to opening

        //if watchdog timer is running and ir sensor is still triggered
        }else if(wdTimer.is_running() && readPin(GPIOA, 3)){ 
            wdTimer.kick();                     //kick watchdog timer
            queue.call(lid_rise_ir_handler);    //add rise_handler to queue
        }

    //IR_MODE
    }else if(mode == IR_MODE){
        //if the lid is not open, the ir sensor is activated, and the lid isn't opening
        if(!is_lid_open && readPin(GPIOA, 3) && !opening_lid_lock){
            queue.call(openLid, &servo_l, &servo_r, 0, 1);  //add openLid to the queue
            
            opening_lid_lock = true; //set lid to opening
            rise_lock = false;       //set rise handler to out of queue
        
        //if lid is closing and there is no rise handler in the queue
        }else if(closing_lid_lock && !rise_lock){
            queue.call(lid_rise_ir_handler); //add rise handler to the queue
            rise_lock = true;                //set rise handler to in queue
        }else{
            rise_lock = false;  //set rise handler to out queue
        }
    }
}

/*
    This handles the fall interrupt for the ir sensor in the front of the trash can
    There are two modes:
    WATCHDOG_MODE
        It starts the watchdog timer
        once the timer runs out the program restarts

    IR_MODE
        If the IR sensor is deactivated it should close the lid
        If the lid is in the middle of opening the fall handler should be added to the queue so that it runs after the lid opens

    (This IR sensor gives a 0 when there is notheing detected and a 1 when something is)
*/
void lid_fall_ir_handler(){
    //WATCHDOG_MODE
    if(mode == WATCHDOG_MODE){
        //if the lid is open or opening and the watchdog timer hasn't started start it.
        if((is_lid_open || opening_lid_lock) && !wdTimer.is_running()){
            wdTimer.start(TIMEOUT);
        }

    //IR_MODE
    }else if(mode == IR_MODE){
        //if lid is open, ir sensor is deactiviated and lid isn't closing
        if(is_lid_open && !readPin(GPIOA, 3) && !closing_lid_lock){
            queue.call(closeLid, &servo_l, &servo_r, 1000, 1); //add close lid to queue
            
            closing_lid_lock = true;    //set lid to closing
            fall_lock = false;          //set fall handler out of queue

        //if lock is opening and fall handler is not in queue
        }else if(opening_lid_lock && !fall_lock){
            queue.call(lid_fall_ir_handler); //add fall handler to queue
            fall_lock = true; //set fall handler to in queue
        }else{
            fall_lock = false; //set fall handler to in queue
        }
    }
}

/*
    This handles the interrupt for the ir sensor inside the trash can.
    The the ir sensor is triggered the led on the trash can should light up
    (The IR sensor gives a 1 when there is notheing detected and a 0 when something is)
*/
void trash_ir_handler(){
    //it s reading the pin for the ir sensor inverting it and writing it to the led pin
    writePin(GPIOC, 3, !readPin(GPIOC, 0));
}

/*
    This handles the interrupt for the push button
    This switches the mode of the trash can
*/
void button_handler(){
    mode = !mode;
}


//-------------------Drivers-------------------//
/*
    Inputs
*/
void initLid(PwmOut* leftServo, PwmOut* rightServo){
    rightServo->period_ms(50);
    leftServo->period_ms(50);

    //closed
    rightServo->pulsewidth(.0005);
    leftServo->pulsewidth(.0015);
}

/*
    Inputs
*/
void openLid(PwmOut* leftServo, PwmOut* rightServo, uint32_t time, uint32_t res){
    int delay = time / res;
    float increment = .001 / res;
    float rightPulseWidth = .0005;
    float leftPulseWidth = .0015;

    //closed
    rightServo->pulsewidth(rightPulseWidth);
    leftServo->pulsewidth(leftPulseWidth);

    for(int i = 0; i < res; i++){
        thread_sleep_for(delay);
        rightPulseWidth += increment;
        leftPulseWidth -= increment;
        rightServo->pulsewidth(rightPulseWidth);
        leftServo->pulsewidth(leftPulseWidth);
    }

    opening_lid_lock = false;
    is_lid_open = true;
}

/*
    Inputs
*/
void closeLid(PwmOut* leftServo, PwmOut* rightServo, uint32_t time, int res){
    int delay = time / res;
    float increment = .001 / res;
    float rightPulseWidth = .0015;
    float leftPulseWidth = .0005;

    //open
    rightServo->pulsewidth(rightPulseWidth);
    leftServo->pulsewidth(leftPulseWidth);

    for(int i = 0; i < res; i++){
        thread_sleep_for(delay);
        rightPulseWidth -= increment;
        leftPulseWidth += increment;
        rightServo->pulsewidth(rightPulseWidth);
        leftServo->pulsewidth(leftPulseWidth);
    }

    closing_lid_lock = false;
    is_lid_open = false;
}

/*
    Inputs
    Port: GPIO_TypeDef* Port (ex GPIOB)
    Pin: uint16_t pin number (ex 7)
    Mode: 0 read, 1 write, 2 alternate function, 3 analog

    This function enables the clock for the current port and sets the mode for the given Port and Pin number
*/
void setPin(GPIO_TypeDef* Port, uint16_t Pin, uint8_t Mode){
    RCC->AHB2ENR |= 0x1 << ((Port - GPIOA) / 23);
    Port->MODER &= ~(0x3 << (Pin * 2));
    Port->MODER |= Mode << (Pin * 2);
}

/*
    Inputs
    Port: GPIO_TypeDef* Port (ex GPIOB)
    Pin: uint16_t pin number (ex 7)

    Return
    state: bool 1 or 0

    This function returns a bool for the state in the IDR for the given Port and Pin number.
*/
bool readPin(GPIO_TypeDef* Port, uint16_t Pin){
    return (Port->IDR & (0x1 << Pin)) >> Pin;
}

/*
    Inputs
    Port: GPIO_TypeDef* Port (ex GPIOB)
    Pin: uint16_t pin number (ex 7)
    State: bool 1 or 0

    This function writes the state to the ODR at the given Port and pin number
*/
void writePin(GPIO_TypeDef* Port, uint16_t Pin, bool state){
    Port->ODR &= ~(0x1 << Pin);
    Port->ODR |= (state << Pin);
}

/*
    Inputs
    Port: GPIO_TypeDef* Port (ex GPIOB)
    Pin: uint16_t pin number (ex 7)

    This function flips the state of the ODR at the given Port and pin number
*/
void flipPin(GPIO_TypeDef* Port, uint16_t Pin){
    Port->ODR ^= (0x1 << Pin);
}

/*
    Inputs
    Port: GPIO_TypeDef* Port (ex GPIOB)
    Pin: uint16_t pin number (ex 7)
    Mode: 0 no pull-up pull-down, 1 pull-up, 2 pull-down

    This function sets the pull mode for the given Port and Pin number
*/
void pullPin(GPIO_TypeDef* Port, uint16_t Pin, uint8_t Mode){
    Port->PUPDR &= ~(0x3 << (Pin * 2));
    Port->PUPDR |= Mode << (Pin * 2);
}