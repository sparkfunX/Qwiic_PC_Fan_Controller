#include <Wire.h>
#include <EEPROM.h>
#include <PIDController.h> // https://github.com/DonnyCraft1/PIDArduino

/* PIN ASSIGNMENTS */
#define PIN_TRIM_KI 0    // ADC in from trimpot
#define PIN_TRIM_KP 1    // ADC in from trimpot
#define PIN_FAN_ENABLE 2 // DO to !FAN_EN
#define PIN_PWM_OUT 3    // DO to FAN_PWM
#define PIN_TACH_IN 7    // DI from FAN_TACH

/* I²C STUFF */
#define REG_MAP_SIZE 0x10   // Used to ignore invalid requests
#define MAX_SENT_BYTES 0x05 // Used to ignore invalid commands
#define DEFAULT_ADDR 0x36   // Factory Default Peripheral Address

/* REGISTER MAP */
#define WIRE_ADDR 0x00             // Device Address
#define FAN_ENABLE 0x01            // Enable power to fan VCC pin (0xFF to enable)
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
#define KP_TRIM_SCALING 0x0F       // The ADC value of the KP trimmer is scaled by this value to allow precision in onboard tuning (signed uint8_t)
#define KP_TRIM_OFFSET 0x10        // This value is added to the ADC value of the KP trimmer to allow precision in onboard tuning (signed uint8_t)
#define KI_TRIM_SCALING 0x11       // The ADC value of the KI trimmer is scaled by this value to allow precision in onboard tuning (signed uint8_t)
#define KI_TRIM_OFFSET 0x12        // This value is added to the ADC value of the KI trimmer to allow precision in onboard tuning (signed uint8_t)
#define TRIMMER_DISABLE 0x13       // Stop reading trimpot ADCs and only use values written to the Kx_VALUE register by the I²C Controller (0xFF for Disable)

/* EEPROM MAP */
#define EEPROM_DEV_ADDR 0x00        // Because the I²C device address is user selectable, we need to remember it between boot cycles
#define EEPROM_FIRST_BOOT_FLAG 0x01 // We check this flag on boot and if it's 0xFF then happy birthday, we need to do some first-boot config
#define EEPROM_HARDWARE_CONFIG 0x02 // This firmware is used on several products with different hardware configurations. We can auto-detect \
                                    // which hardware we're running on, but let's only do it on first boot and then try to remember
#define EEPROM_FACTORY_TRIM_KP 0x03 // A place to store the trimpot's factory position
#define EEPROM_FACTORY_TRIM_KI 0x04 // A place to store the trimpot's factory position
#define EEPROM_TRIM_CHANGE 0x05     // Use this flag to avoid number compares in control loop after the trimpots are changed from factory setting

/* HARDWARE CONFIG SIGNATURES */
#define QWIIC_4_WIRE_FAN_CTRL 0xAA // SPX-xxxxx
#define QWIIC_BLOWER 0xBB          // SPX-xxxxx

/* MISCELLANEOUS GLOBAL VARS */
uint8_t registerMap[REG_MAP_SIZE];        // Bus-accessible device registers
uint8_t receivedCommands[MAX_SENT_BYTES]; // I²C command buffer
uint8_t wire_addr = DEFAULT_ADDR;         // I²C device address
bool changeFlag = 0;                      // Use this flag to alert the main loop when a register change requires attention
uint8_t registerIndex = 0;                // Use this to track our position in the device registers during consecutive read operations
uint8_t hardwareMode = 0;                 // Var that reflects the contents of EEPROM_HARDWARE_VER for comparing at a glance during operations

// Instantiate PI Controller
PIDController piController;

void setup()
{
    // Check EEPROM for a bus address and set accordingly
    setWireAddress();

    // load up the register defaults
    setRegisterDefaults();

    // Check EEPROM to see if this is first boot, if so then do some config
    selectHardwareConfig();

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
    piController.limit(0, 320); // Limit the PI output to prevent integral windup
}

void loop()
{
    delay(200);

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
    if (hardwareMode == QWIIC_4_WIRE_FAN_CTRL && registerMap[TRIMMER_DISABLE] != 0xFF)
    {
        readTrimmers();
    }

    // Check whether the user has disabled the control loop in
    // favor of direct proportional speed control. If so, set
    // the PWM Width according to the contents of PROPORTIONAL_THROTTLE
    // if not, compute the output of the PI controller from the
    // contents of SETPOINT_RPM
    if (PI_LOOP_DISABLE == 0xFF)
    {
        pwmWidth(map(registerMap[PROPORTIONAL_THROTTLE], 0, 100, 0, 320));
    }
    else
    {
        piUpdate();
    }
}

/****************************************************
Function piUpdate()
The PID library has a 'compute' function, which we do
call in this method. However, in our particular case, 
it's easier to package that up with the tuning functions
becuase we always update our tunings before calculating 
the output. It seems excessive but it costs us less time
to just update the tunings than it does to check whether
it's necessary. We also use this method to update the
output value in the register map. 
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

    // Update the PI Controller
    piController.setpoint(setpoint);
    piController.tune(proportional, integral, 0);
    piOutput = piController.compute(tachReading);

    // Update the PI_OUT register entry
    registerMap[PI_OUT] = piOutput >> 8;
    registerMap[PI_OUT + 0x01] = piOutput;

    // Actually set the fan PWM output
    pwmWidth(piOutput);
}

/****************************************************
Function readTrimmerAndTune()
Read the Trimpots, process them according to the 
scaling and offset registers, constrain them to 
the size of the Kx_VALUE registers, and store 
them for reference when tuning the PI Controller
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
    ADCKP *= static_cast<int8_t>(registerMap[KP_TRIM_SCALING]) * 0.1; // Range -12.8 to 12.7
    ADCKI *= static_cast<int8_t>(registerMap[KI_TRIM_SCALING]) * 0.1; // Range -12.8 to 12.7

    // Add the offset in Kx_TRIM_OFFSET
    // Use reinterpret_cast to ensure our int8 is correctly extracted from uint8 registerMap[]
    ADCKP += static_cast<int8_t>(registerMap[KP_TRIM_OFFSET]);
    ADCKI += static_cast<int8_t>(registerMap[KI_TRIM_OFFSET]);

    // ADCKx could very well be too large for int16 Kx_VALUE but because we used the correct
    // precision in all of the math up until now, we can simply truncate the result and all
    // we lose are a few decimals of precision that we don't want to store anyway
    registerMap[KP_VALUE] = int16_t(ADCKP * 100) >> 8;
    registerMap[KP_VALUE + 0x01] = int16_t(ADCKP * 100);
    registerMap[KI_VALUE] = int16_t(ADCKI * 100) >> 8;
    registerMap[KI_VALUE + 0x01] = int16_t(ADCKI * 100);
}

/****************************************************
Function selectHardwareConfig()
This firmware is used on two different products
that are nearly identical aside from some included
hardware. The first time the hardware boots, we 
need to determine which one we're on. We can take
advantage of the fact that pins are floating on 
some boards and on others they're connected to 
trimpots. Once we know what hardware we're running on
we can save that info in EEPROM and we don't need to 
check again. The hardware config is user selectable 
just in case we get it wrong or they want to fiddle 
around with it.
****************************************************/
void selectHardwareConfig()
{
    // Check EEPROM for FIRST_BOOT_FLAG
    uint8_t eepromVal;
    EEPROM.get(EEPROM_FIRST_BOOT_FLAG, eepromVal);
    if (eepromVal == 0xFF) // If the EEPROM has never been touched, it will read 0xFF
    {
        // This is first boot. Now that we're here, set the flag
        EEPROM.put(EEPROM_FIRST_BOOT_FLAG, 0x00);

        // Now we need to determine which hardware config to set up for and store it in EEPROM
        // To do this, we'll check if pin A0 is floating. On the Qwiic_4-Wire_Fan_Controller,
        // this pin is connected to a trimpot, but on the Qwiic_Blower, there is no trimpot so
        // the pin is left floating.
        if (detectFloat(A0))
        {
            EEPROM.put(EEPROM_HARDWARE_CONFIG, QWIIC_BLOWER);
        }
        else
        {
            EEPROM.put(EEPROM_HARDWARE_CONFIG, QWIIC_4_WIRE_FAN_CTRL);
            // Since we know that there are trimpots connected, we now need to read them so that
            // we know when a user changes them. Their position is not guaranteed after assembly
            // and we would like to start the device with known reasonable PI controller settings
            // so by reading the trimpots on first boot, we can then ignore them if they're unchanged
            // and simply use the programmed defaults

            // We divide the 10-bit ADC reading by 4 to give us something we can store in a single uint8_t
            // We don't want to be too sensitive to changes on the ADC when detecting whether the
            // trimpot has ever been touched, so in this case stuffing the ADC result into a
            // single uint8_t gives us 0.4% tolerance for free
            EEPROM.put(EEPROM_FACTORY_TRIM_KP, uint8_t(analogRead(PIN_TRIM_KP) / 4));
            EEPROM.put(EEPROM_FACTORY_TRIM_KI, uint8_t(analogRead(PIN_TRIM_KI) / 4));
        }
    }

    // Now we can check our hardware configuration because we've either just confirmed it,
    // or we checked it on first boot
    EEPROM.get(EEPROM_HARDWARE_CONFIG, eepromVal);
    switch (eepromVal)
    {
    case QWIIC_4_WIRE_FAN_CTRL:
        hardwareMode = QWIIC_4_WIRE_FAN_CTRL;
        break;

    case QWIIC_BLOWER:
        hardwareMode = QWIIC_BLOWER;
        break;

    default:
        // This is a weird case where something has gone wrong. We'll act like a
        // QWIIC_4_WIRE_FAN_CTRL for now, but we'll also release the EEPROM_FIRST_BOOT_FLAG
        // so we can try to identify the hardware again on next boot.
        EEPROM.put(EEPROM_FIRST_BOOT_FLAG, 0xFF);
        hardwareMode = QWIIC_4_WIRE_FAN_CTRL;

        break;
    }
}

/****************************************************
Function updateSettings()
Check that our devicce settings still match those 
found in the register map. If not, change the device
settings then reset the change flag. 
This gets called every loop directly after a device 
register is written to. 
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
Function setWireAddress()
Check if there is a device address stored in EEPROM
If there is, set it. If not, store the factory default
and set that.
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
Function readTachometer()
Read the pulse length of the Fan Tachometer and 
convert to real RPM. Cast to 2 bytes and store in 
appropriate register location
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
Function setRegisterDefaults()
The register space lives in RAM. This means we 
need to load the defaults on boot. 
****************************************************/
void setRegisterDefaults()
{
    registerMap[FAN_ENABLE] = 0x00;   // Disable fan on startup to avoid hard start
    registerMap[SETPOINT_RPM] = 0x00; // 0
    registerMap[SETPOINT_RPM + 0x01] = 0x00;
    registerMap[KP_VALUE] = (int)75 >> 8;    // 7.5
    registerMap[KP_VALUE + 0x01] = (int)75;  // 7.5
    registerMap[KI_VALUE] = (int)100 >> 8;   // 10.0
    registerMap[KI_VALUE + 0x01] = (int)100; // 10.0
    registerMap[FAN_TACH_DIVIDER] = 0x02;    // 2 Pulses per Cycle
    registerMap[PI_LOOP_DISABLE] = 0x00;     // PI Controller Enabled
    registerMap[KP_TRIM_SCALING] = 0x01;     // Scale by 1
    registerMap[KP_TRIM_OFFSET] = 0x00;      // No Offset
    registerMap[KI_TRIM_SCALING] = 0x01;     // Scale by 1
    registerMap[KI_TRIM_OFFSET] = 0x00;      // No Offset
    registerMap[TRIMMER_DISABLE] = 0x00;     // Enabled
}

/****************************************************
Function fastPWMSetup()
Setup the appropriate timer registers on the Attiny841
to output FastPWM on OC1B, compare to ICR1 for TOP.
In FastPWM mode, TCNT1 will count up to TOP and then
reset. Output Compare is cleared when they match. 
ICR1 value for 25kHz output frequency derived by:
    16MHz I/O Clock / ( Prescaler 1 x TOP + 1 ) 
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
Function detectFloat()
This firmware is used on two different products
that are nearly identical aside from some included
hardware. To discriminate between them, we need to 
check whether certain pins are left floating. 
This function toggles the mode of any ADC connected
pin to attempt to determine whether it can be
influenced. If the pin doesn't stay in any particular
state after being driven high or pulled low, we 
assume that it's floating.
****************************************************/
bool detectFloat(uint8_t pin)
{
    uint8_t vote = 0;         // Votes for floating
    float tries = 10;         // Measurements to vote on, MUST BE EVEN
    uint16_t threshold = 200; // Threshold to account for ADC charge/discharge rate
    float majority = 0.8;     // What counts as a passing majority in the vote
    uint16_t reading;         // We read the adc into a var just in case we want to peek at it for debugging

    for (uint8_t i = 0; i < (tries / 2); i++)
    {                              // For half the number of tries (because we actually do 2 per loop)
        pinMode(pin, OUTPUT);      // put the pin in output mode
        digitalWrite(pin, 0);      // pull it down
        delay(10);                 // give it time to discharge (it should be near instantaneous)
        pinMode(pin, INPUT);       // prepare to read the ADC, release the pull down
        reading = analogRead(pin); // read the ADC
        if (reading < threshold)
        { // if the ADC was still below threshold, vote for "floating"
            vote++;
        }
        pinMode(pin, OUTPUT);      // put the pin in output mode
        digitalWrite(pin, 1);      // drive the pin high
        delay(10);                 // give it time to charge (it should be near instantaneous)
        pinMode(pin, INPUT);       // prepare to read the ADC
        digitalWrite(pin, 0);      // release the pull up
        reading = analogRead(pin); // read the ADC
        if (reading > (1024 - threshold))
        { // if the ADC is still above (max-threshold) then vote "floating"
            vote++;
        }
    }

    // Tally the votes
    if ((1 / tries) * vote > majority)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/****************************************************
Function requestEvent()
Handle requests from the I²C controller
Emulate a proper I²C peripheral by writing out the 
contents of the last received register index and 
then increment the index for consecutive reads.
Register index is reset whenever new bytes are 
received so it can't run away.
****************************************************/
void requestEvent()
{
    Wire.write(registerMap[registerIndex]);
    registerIndex++;
}

/****************************************************
Function receiveEvent(int bytesReceived)
Handle incoming bytes from I²C controller
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

    if (bytesReceived == 1 && (receivedCommands[0] < REG_MAP_SIZE)) //if we got a uint8_t within the register map range, keep it
    {
        registerIndex = receivedCommands[0];
        return;
    }

    if (bytesReceived == 1 && (receivedCommands[0] >= REG_MAP_SIZE)) //if we got a uint8_t outside the register map range, chuck it
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
        for (int i = 0; i < bytesReceived; i++)
        {
            registerMap[receivedCommands[0] + i] = receivedCommands[i];
        }
        changeFlag = 1; // Set the change flag
    }

    return;
}
