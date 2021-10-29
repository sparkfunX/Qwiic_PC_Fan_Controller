#include <Wire.h>
#include <EEPROM.h>
#include <PIDController.h> // https://github.com/DonnyCraft1/PIDArduino

/****************************************************
 * HARDWARE CONFIGURATION
 * This firmware is used on multiple boards with minor
 * hardware differences so BE CAREFUL to define the 
 * correct hardware configuration below before 
 * programming!
****************************************************/
//#define QWIIC_PC_FAN_CONTROLLER // SPX-18570
#define QWIIC_BLOWER // SPX-18561

#ifdef QWIIC_PC_FAN_CONTROLLER
#define TRIM_DISABLE_DEFAULT 0x00
#endif
#ifdef QWIIC_BLOWER
#define TRIM_DISABLE_DEFAULT 0xFF
#endif 

/* PIN ASSIGNMENTS */
#define PIN_TRIM_KI 0    // ADC in from trimpot
#define PIN_TRIM_KP 1    // ADC in from trimpot
#define PIN_FAN_ENABLE 2 // DO to !FAN_EN
#define PIN_PWM_OUT 3    // DO to FAN_PWM
#define PIN_TACH_IN 7    // DI from FAN_TACH

/* I²C STUFF */
#define MAX_SENT_BYTES 0x05 // Used to ignore invalid commands
#define DEFAULT_ADDR 0x36   // Factory Default Peripheral Address

/* REGISTER MAP */
#define WIRE_ADDR 0x00             // Device Address
#define FAN_ENABLE 0x01            // Enable power to fan VCC pin (0xFF to disable)
#define SETPOINT_RPM 0x02          // PI Controller setpoint in RPM (2 bytes unsigned integer)
#define KP_VALUE 0x04              // PI Controller Proportional Term (2 bytes signed fixed decimal) \
                                   // Setting this parameter from the I²C Controller overrides the onboard trimpot
#define KI_VALUE 0x06              // PI Controller Integral Term (2 bytes signed fixed decimal) \
                                   // Setting this parameter from the I²C Controller overrides the onboard trimpot
#define FAN_TACH_DIVIDER 0x08      // Pulses per rotation indicated by fan/blower datasheet (usually 2)
#define FAN_ACTUAL_RPM 0x09        // Fan Speed Actual as indicated by Tachometer, input to PI Controller (2 bytes integer)
#define PI_OUT 0x0B                // Output of PI Controller for debugging (2 bytes integer)
#define PI_LOOP_DISABLE 0x0D       // Disable PI Controller and use contents of PROPORTIONAL_THROTTLE as speed (0xFF for Disable)
#define PROPORTIONAL_THROTTLE 0x0E // Fan speed as % of MAX. Inverse of PWM Width Cycle.                          \
                                   // When PI Controller is Enabled this register contains its output x (100/320) \
                                   // Whem PI Controller is Disabled the PWM Width Cycle is set to this register's inverse
#define TRIMMER_SCALE 0x0F         // Trimpot values are multiplied by this number. Probably only useful in edge cases. 2.0 (20) by default.
#define TRIMMER_DISABLE 0x10       // Stop reading trimpot ADCs and only use values written to the Kx_VALUE register by the I²C Controller (0xFF for Disable)

/* EEPROM MAP */
#define EEPROM_DEV_ADDR 0x00        // Because the I²C device address is user selectable, we need to remember it between boot cycles

/* MISCELLANEOUS GLOBAL VARS */
uint8_t receivedCommands[MAX_SENT_BYTES]; // I²C command buffer
uint8_t wire_addr = DEFAULT_ADDR;         // I²C device address
bool changeFlag = 0;                      // Use this flag to alert the main loop when a register change requires attention
uint8_t registerIndex = 0;                // Use this to track our position in the device registers during consecutive read operations

/****************************************************
 * SET THE REGISTER MAP DEFAULTS
 * These are the bus-accessible device registers
 * and their default values */
uint8_t registerMap[0x11] = 
    {
        0x00,                   // WIRE_ADDR
        0x00,                   // FAN_ENABLE
        0x00,                   // SETPOINT_RPM
        0x00,                   // SETPOINT_RPM BYTE 2
        0,                      // KP_VALUE (250 interpreted as 2.50)
        250,                     // KP_VALUE BYTE 2 
        1,                      // KP_VALUE (390 interpreted as 3.90)
        134,                     // KP_VALUE BYTE 2
        2,                   // FAN_TACH_DIVIDER (2 pulses per rotation)
        0x00,                   // FAN_ACTUAL_RPM 
        0x00,                   // FAN_ACTUAL_RPM BYTE 2
        0x00,                   // PI_OUT
        0x00,                   // PI_OUT BYTE 2
        0x00,                   // PI_LOOP_DISABLE (enabled)
        0x00,                   // PROPORTIONAL_THROTTLE
        20,                   // TRIMMER_SCALE (20 interpreted as 2.0)
        TRIM_DISABLE_DEFAULT    // TRIMMER_DISABLE
    };
/****************************************************/

// Instantiate PI Controller
PIDController piController;

void setup()
{
    // Check EEPROM for a bus address and set accordingly
    setWireAddress();

    // Setup timers for 25kHz Fast PWM
    fastPWMSetup();

    // Set the initial pin states
    pinMode(PIN_PWM_OUT, OUTPUT);
    pinMode(PIN_TACH_IN, INPUT_PULLUP); // Fan Tachometer is open-collector
    pinMode(PIN_FAN_ENABLE, OUTPUT);
    digitalWrite(PIN_FAN_ENABLE, HIGH); // Fan Enable is active low. Boot disabled to avoid hard start
    pinMode(PIN_TRIM_KI, INPUT);
    pinMode(PIN_TRIM_KP, INPUT);

    // Start I2C
    Wire.begin(wire_addr);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);

    // Start the PI Controller
    piController.begin();
    piController.limit(20, 320); // Limit the PI output to prevent integral windup
}

void loop()
{
    // Get the tachometer reading, turn it into a real RPM using the value from
    // FAN_TACH_DIVIDER, and stuff it into FAN_ACTUAL_RPM for the I²C controller
    // to query and for the PI controller to compute against.
    readTachometer();

    // Finding which device register has been changed and updating
    // the approptiate settings takes some time so we only do it if
    // the registers have been poked. This way, we interfere as little
    // as possible with the PI Controller and PWM Generator
    if (changeFlag)
    {
        updateSettings();
    }

    // If the hardware is a Qwiic 4-Wire Fan Controller and the TRIMMER_DISABLE flag isn't set
    // then read the trimmers, apply the correct scale and offset, and then load those
    // values into the Kx_VALUE registers. If the TRIMMER_DISABLE flag is set, or this is
    // a Qwiic Blower board without trimpots, we skip reading the trimpots, so whatever
    // value is in the Kx_VALUE register from the last I²C receive is used for tuning.
    if (registerMap[TRIMMER_DISABLE] != 0xFF)
    {
        readTrimmers();
    }

    // Check whether the user has disabled the control loop in
    // favor of direct proportional speed control. If so, set
    // the PWM Width according to the contents of PROPORTIONAL_THROTTLE
    // if not, compute the output of the PI controller from the
    // contents of SETPOINT_RPM
    if (registerMap[PI_LOOP_DISABLE] == 0xFF)
    {
        pwmWidth(map(registerMap[PROPORTIONAL_THROTTLE], 0, 100, 0, 320));
    }
    else
    {
        piUpdate();
    }
}

/****************************************************
 * Function piUpdate()
 * The PID library has a 'compute' function, which we do
 * call in this method. However, in our particular case, 
 * it's easier to package that up with the tuning functions
 * becuase we always update our tunings before calculating 
 * the output. It seems excessive but it costs us less time
 * to just update the tunings than it does to check whether
 * it's necessary. We also use this method to update the
 * output value in the register map. 
****************************************************/
void piUpdate()
{

    float proportional = 0;
    float integral = 0;
    uint16_t setpoint = 0;
    uint16_t piOutput = 0;
    uint16_t tachReading = 0;

    // Expand the registerMap values back into int16
    // Multiply by 0.01 as neccessary to move the decimal into
    // the right position
    proportional += registerMap[KP_VALUE] << 8;
    proportional += registerMap[KP_VALUE + 0x01];
    proportional *= 0.01;

    integral += registerMap[KI_VALUE] << 8;
    integral += registerMap[KI_VALUE + 0x01];
    integral *= 0.01;

    setpoint += registerMap[SETPOINT_RPM] << 8;
    setpoint += registerMap[SETPOINT_RPM + 0x01];

    tachReading += registerMap[FAN_ACTUAL_RPM] << 8;
    tachReading += registerMap[FAN_ACTUAL_RPM + 0x01];

    // PC Fans tend to have a dead zone at the low end, so we limit the PI controller
    // to the range above this dead zone (to prevent stalling the fan and setting up
    // an oscillation) This does mean, however, that the PI controller will never 
    // organically output a 0, so if the fan speed is set to 0 we need to override the
    // PI controller
    if(setpoint !=0){

      // Update the PI Controller
      piController.setpoint(setpoint);
      piController.tune(proportional, integral, 0);
      piOutput = piController.compute(tachReading);

    }else{

      piOutput = 0;
      
    }

    // Update the PI_OUT register entry
    registerMap[PI_OUT] = piOutput >> 8;
    registerMap[PI_OUT + 0x01] = piOutput;

    // Actually set the fan PWM output
    pwmWidth(piOutput);
}

/****************************************************
 * Function readTrimmerAndTune()
 * Read the Trimpots, process them according to the 
 * scaling and offset registers, constrain them to 
 * the size of the Kx_VALUE registers, and store 
 * them for reference when tuning the PI Controller
****************************************************/
void readTrimmers()
{

    // Read ADCs into some temp vars
    // The data types are a little weird because of trying to make them
    // as small as possible for I²C transmission. This results in some
    // funny float math
    float ADCKP = analogRead(PIN_TRIM_KP) * 0.01; // Range 0 - 10.24
    float ADCKI = analogRead(PIN_TRIM_KI) * 0.01; // Range 0 - 10.24

    // Multiply by the scaling value from Kx_TRIM_SCALING
    // Use reinterpret_cast to ensure our int8 is correctly extracted from uint8 registerMap[]
    ADCKP *= static_cast<int8_t>(registerMap[TRIMMER_SCALE]) * 0.1; // Range -12.8 to 12.7
    ADCKI *= static_cast<int8_t>(registerMap[TRIMMER_SCALE]) * 0.1; // Range -12.8 to 12.7

    // ADCKx could very well be too large for int16 Kx_VALUE but because we used the correct
    // precision in all of the math up until now, we can simply truncate the result and all
    // we lose are a few decimals of precision that we don't want to store anyway
    registerMap[KP_VALUE] = int16_t(ADCKP * 100) >> 8;
    registerMap[KP_VALUE + 0x01] = int16_t(ADCKP * 100);
    registerMap[KI_VALUE] = int16_t(ADCKI * 100) >> 8;
    registerMap[KI_VALUE + 0x01] = int16_t(ADCKI * 100);
}

/****************************************************
 * Function updateSettings()
 * Check that our devicce settings still match those 
 * found in the register map. If not, change the device
 * settings then reset the change flag. 
 * This gets called every loop directly after a device 
 * register is written to. 
****************************************************/
void updateSettings()
{

    // Check if the device address has been changed
    // Compare against global var wire_addr which is
    // unchanged since last Wire.begin() call
    if (registerMap[WIRE_ADDR] != wire_addr)
    {
        wire_addr = registerMap[WIRE_ADDR];
        // store the I²C Address in EEPROM so it sticks around on reboot
        EEPROM.put(EEPROM_DEV_ADDR, wire_addr);
        // restart I²C
        Wire.end();
        Wire.begin(wire_addr);
        Wire.onRequest(requestEvent);
        Wire.onReceive(receiveEvent);
    }

    // The following are easy to evaluate and don't
    // touch EEPROM so we go ahead and "update" them
    // every time whether they've been changed or not

    // FAN ENABLE
    if (registerMap[FAN_ENABLE] == 0xFF)
    {
        digitalWrite(PIN_FAN_ENABLE, HIGH);
    }
    else
    {
        digitalWrite(PIN_FAN_ENABLE, LOW);
    }

    // Reset the flag
    changeFlag = 0;
}

/****************************************************
 * Function setWireAddress()
 * Check if there is a device address stored in EEPROM
 * If there is, set it. If not, store the factory default
 * and set that.
****************************************************/
void setWireAddress()
{
    uint8_t eepromVal;
    EEPROM.get(EEPROM_DEV_ADDR, eepromVal);
    if (eepromVal == 0xFF)
    {
        EEPROM.put(EEPROM_DEV_ADDR, DEFAULT_ADDR);
        registerMap[WIRE_ADDR] = DEFAULT_ADDR;
    }
    else
    {
        wire_addr = eepromVal;
        registerMap[WIRE_ADDR] = eepromVal;
    }
}

/****************************************************
 * Function readTachometer()
 * Read the pulse length of the Fan Tachometer and 
 * convert to real RPM. Cast to 2 bytes and store in 
 * appropriate register location
****************************************************/
void readTachometer()
{
    long tach; // Pulse length of tachometer pulse in microseconds
    long freq; // Frequency of tachometer pulse in Hz derived by (1000000 / pulse length in microseconds) / 2 transitions per pulse
    long rpm;  // RPM derived by (frequency / pulses per revolution) x 60

    tach = pulseIn(PIN_TACH_IN, LOW); // Get pulse length of tachometer pulse, from this we can derive the fan speed
    freq = 1000000 / tach / 2;
    rpm = (freq / registerMap[FAN_TACH_DIVIDER]) * 60;

    registerMap[FAN_ACTUAL_RPM] = uint16_t(rpm) >> 8;
    registerMap[FAN_ACTUAL_RPM + 0x01] = uint16_t(rpm);
}

/****************************************************
 * Function fastPWMSetup()
 * Setup the appropriate timer registers on the Attiny841
 * to output FastPWM on OC1B, compare to ICR1 for TOP.
 * In FastPWM mode, TCNT1 will count up to TOP and then
 * reset. Output Compare is cleared when they match. 
****************************************************/
void fastPWMSetup()
{
    TCCR1B = 0; // Clear Timer Control Register 1B
    TCNT1 = 0;  // Clear Timer1

    TCCR1A &= ~_BV(WGM10); //FastPWM mode with ICR1
    TCCR1A |= _BV(WGM11);
    TCCR1A |= _BV(WGM12);
    TCCR1B |= _BV(WGM13);

    TCCR1A |= _BV(COM1B1); // Compare Output Mode Channel B (Non-Inverting Mode)
    TCCR1A &= ~_BV(COM1B0);

    TCCR1B |= _BV(CS10); // No Prescaler
    TCCR1B &= ~_BV(CS11);
    TCCR1B &= ~_BV(CS12);

    ICR1 = 320; //FastPWM mode counts up 320 (25kHz)
    OCR1B = 0;  //0-320 = 0-100% Width cycle
}

void pwmWidth(int width)
{
    // Change the width of the PWM wave generator
    // Maximum width is 320 cycles (100% duty)
    OCR1B = constrain(width, 0, 320);
    // Store the speed that we're setting the fan
    // as a proportion of the maximum (essentially
    // the inverse of the PWM duty cycle)
    registerMap[PROPORTIONAL_THROTTLE] = uint8_t(map(width, 0, 320, 0, 100));
}

/****************************************************
 * Function requestEvent()
 * Handle requests from the I²C controller
 * Emulate a proper I²C peripheral by writing out the 
 * contents of the last received register index and 
 * then increment the index for consecutive reads.
 * Register index is reset whenever new bytes are 
 * received so it can't run away.
****************************************************/
void requestEvent()
{
  for(uint8_t idx = 0; idx < sizeof(registerMap); idx++)
  {
    Wire.write(registerMap[registerIndex+idx]);
  }
}

/****************************************************
 * Function receiveEvent(int bytesReceived)
 * Handle incoming bytes from I²C controller
****************************************************/
void receiveEvent(int bytesReceived)
{

    /* If we receive a single uint8_t, we can assume the controller is asking for the contents of a register 
   so we store the uint8_t in a global var that we can retreive during requestEvent() */

    for (int a = 0; a < bytesReceived; a++)
    {
        if (a < MAX_SENT_BYTES)
        {
            receivedCommands[a] = Wire.read(); //grab stuff and jam it into the command buffer
        }
        else
        {
            Wire.read(); //if we receive more data than allowed, chuck it
        }
    }

    if (bytesReceived == 1 && (receivedCommands[0] < sizeof(registerMap))) //if we got a uint8_t within the register map range, keep it
    {
        registerIndex = receivedCommands[0];
        return;
    }

    if (bytesReceived == 1 && (receivedCommands[0] >= sizeof(registerMap))) //if we got a uint8_t outside the register map range, chuck it
    {
        receivedCommands[0] = 0x00;
        registerIndex = 0x00;
        return;
    }

    /* If we receive multiple bytes, the first is a register address and the following 
    are values to write starting at the specified address */

    if (bytesReceived > 1)
    {
        // load received bytes into consecutive register addresses
        for (int i = 0; i < bytesReceived - 1; i++)
        {
            registerMap[receivedCommands[0] + i] = receivedCommands[1 + i];
        }
        changeFlag = 1; // Set the change flag
    }

    return;
}
