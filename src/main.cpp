#include <Arduino.h>

extern "C" { //Needed for VESC libraries since they're compiled as C++ but written in C
#include <bldc_interface_uart.h>
#include <datatypes.h>
#include <bldc_interface.h>
#include <buffer.h>
}

#include <Wire.h>

/**
 * This code uses the following global naming prefix conventions:
 * k - A constant.
 * c - A current value.  This is used on variables to represent the most most up to date version of a particular measurement
 * t - A time storage variable.
 * VESC - A function or variable for interacting with the VESC motor controller
 * HMI - A function or variable for interacting with the HMI controller
 * SC - A function or variable for interacting with the safety controller
 */

//Constants
#define kLOOP_RATE_MICROS 1000 //Rate the loop should run at
#define kVESC_BAUDRATE 115200 //The baudrate configured in the VESC

//Rate parameters
#define kVESC_WRITE_RATE_MS 10
#define kVESC_READ_RATE_MS 100
#define kHMI_UPDATE_RATE_MS 100
#define kSC_UPDATE_RATE_MS 100

//Addresses
#define kSC_WIRE_ADDR 21
#define kHMI_WIRE_ADDR 22

//Geometry and conversions
#define kFINAL_DRIVE_REDUCTION 1.5 //gear ratio
#define kDRIVE_WHEEL_CIRCUMFERENCE 23.25 //inches
#define kACCELERATION_MPH_PER_SECOND 3.25 //mph/s
#define kOUTPUT_POLARITY 1 //-1 to invert output, 1 for normal output, 0 to disable output (VESC)

//Communication constnats
#define kHMI_REQ_SIZE 4 //how many bytes the HMI controller will send us (4 as of 12/18/2019)
#define kHMI_REP_SIZE 22 //how many bytes we send the HMI controller (22 as of 12/18/2019)

double convertERPMtoMPH(int32_t erpm) {
    return (erpm * kFINAL_DRIVE_REDUCTION * kDRIVE_WHEEL_CIRCUMFERENCE * 60.0) / 4435200.0;
}

int convertMPHtoERPM(double mph) {
    return (int) ((mph * 4435200.0) / (kFINAL_DRIVE_REDUCTION * kDRIVE_WHEEL_CIRCUMFERENCE * 60.0));
}

int kVELOCITY_DV_ERPM = (int) (convertMPHtoERPM(kACCELERATION_MPH_PER_SECOND) / (1000.0 / kVESC_WRITE_RATE_MS));


//Runtime values
unsigned long ct = 0; //Current time

float cVESC_inputVoltage = 0.0f;
float cVESC_currentDrawAmps = 0.0f;
float cVESC_ERPM = 0.0f;

int cHMI_speedSetpointERPM = 0;

bool cSC_estopActive = false;

int c_activeSpeedTargetERPM = 0;

uint8_t cHMI_readBuffer[kHMI_REQ_SIZE];
uint8_t cHMI_writeBuffer[kHMI_REP_SIZE];

//Runtime functions
/**
 * Writes data to the serial interface (UART) that is connected to the VESC
 * @param data The pointer to the bytes to write
 * @param length The length of the array of data at the pointer
 */
void VESC_serialWrite(unsigned char *data, unsigned int length) {
    Serial1.write(data, length);
}

/**
 * Processes incoming serial data from the VESC
 */
void VESC_processIncomingData() {
    //Use a for loop capped at 4096 to prevent us from entirely blocking our main loop if a ton of data
    //floods the bus.  This loop ends when either there is no more data left or we hit the limit.
    for (int i = 0; Serial1.available() && i < 4096; i++) {
        bldc_interface_uart_process_byte(Serial1.read());
    }
}

/**
 * Function called when new data is received from the VESC
 * @param values The values received
 */
void VESC_onValuesUpdated(mc_values *values) {
    //Store the values from the VESC
    cVESC_inputVoltage = values->v_in;
    cVESC_currentDrawAmps = values->current_motor;
    cVESC_ERPM = values->rpm;
}

/**
 * Requests an update from the safety controller
 */
void SC_update() {
    Wire.requestFrom(kSC_WIRE_ADDR, 1); //This clears the Wire RX buffer for us so no need to worry about other data in the buffer.
    auto byteIn = Wire.read();
    cSC_estopActive = byteIn != 0b00000000;
}

/**
 * Requests an update from the HMI controller, and sends the HMI controller current state information
 */
void HMI_update() {
    Wire.requestFrom(kHMI_WIRE_ADDR, kHMI_REQ_SIZE); //Request from the HMI
    Wire.readBytes(cHMI_readBuffer, kHMI_REQ_SIZE); //Read the data from the HMI

    int32_t i = 0; //Buffer index, automatically incremented by buffer read and append calls

    //Read data from buffer
    //float32 speed_setpoint (MPH)
    //TODAL - 32 bits (4 bytes)
    //Note that there is only one float currently, but we use a buffer here to make it easy to add more data later
    float speedSetpointMph = buffer_get_float32_auto(cHMI_readBuffer, &i);

    //Set current values
    cHMI_speedSetpointERPM = convertMPHtoERPM(speedSetpointMph);

    //Convert values for sending
    auto speedMph = (float) convertERPMtoMPH(cVESC_ERPM);
    auto speedTargetMph = (float) convertERPMtoMPH(c_activeSpeedTargetERPM);

    i = 0; //Reset buffer index for writing

    //Write data to out buffer
    //float32 battery_voltage
    //uint8 battery_state
    //float32 current_draw
    //float32 speed (MPH)
    //float32 speed_target (MPH)
    //int8 estop_state
    //int32 fault_code
    //TOTAL - 176 bits (22 bytes)
    buffer_append_float32_auto(cHMI_writeBuffer, cVESC_inputVoltage, &i);
    buffer_append_uint8(cHMI_writeBuffer, 0, &i); //TODO battery state
    buffer_append_float32_auto(cHMI_writeBuffer, cVESC_currentDrawAmps, &i);
    buffer_append_float32_auto(cHMI_writeBuffer, speedMph, &i);
    buffer_append_float32_auto(cHMI_writeBuffer, speedTargetMph, &i);
    buffer_append_uint8(cHMI_writeBuffer, cSC_estopActive, &i);
    buffer_append_int32(cHMI_writeBuffer, 0, &i); //TODO fault code


    //Write buffer out to wire
    Wire.beginTransmission(kHMI_WIRE_ADDR);
    Wire.write(cHMI_writeBuffer, kHMI_REP_SIZE);
    Wire.endTransmission();
}

/**
 * Updates the velocity target to be sent to the VESC based on the current setpoint and the acceleration parameter
 */
void updateSpeed() {
    if (cHMI_speedSetpointERPM > c_activeSpeedTargetERPM) {
        //Setpoint is faster than current target, speed up
        c_activeSpeedTargetERPM += kVELOCITY_DV_ERPM;
        if (c_activeSpeedTargetERPM > cHMI_speedSetpointERPM) {
            //This add put us over the setpoint, set the current target equal to the setpoint
            c_activeSpeedTargetERPM = cHMI_speedSetpointERPM;
        }
    } else if (cHMI_speedSetpointERPM < c_activeSpeedTargetERPM) {
        //Setpoint is slower than current target, slow down
        c_activeSpeedTargetERPM -= kVELOCITY_DV_ERPM;
        if (c_activeSpeedTargetERPM < cHMI_speedSetpointERPM) {
            //This subtract put us under the setpoint, set the current target equal to the setpoint
            c_activeSpeedTargetERPM = cHMI_speedSetpointERPM;
        }
    }
}

/**
 * Resets the target speed to zero immediately and resets the setpoint to zero.  This gets called when an e-stop
 * occurs.
 */
void stopAndResetSpeed() {
    c_activeSpeedTargetERPM = 0;
    cHMI_speedSetpointERPM = 0;
}

/**
 * Sends a command to the VESC based on the current state.
 */
void VESC_write() {
    bldc_interface_set_rpm(kOUTPUT_POLARITY * c_activeSpeedTargetERPM);
}

void setup() {
    //Serial is the UART connected to the USB port, Serial1 is the UART connected to the pins on the board.
    //Serial will be used for debug output, Serial1 will be used to communicate with the VESC

    Serial.begin(115200); //Used for debugging over USB
    Serial1.begin(kVESC_BAUDRATE); //Used to communicate with the VESC
    Wire.begin(); //Used to communicate with other controllers (safety and HMI)

    bldc_interface_uart_init(VESC_serialWrite); //This binds the "serialWriteToVesc" function to the VESC library.
    bldc_interface_set_rx_value_func(VESC_onValuesUpdated); //Binds the "onVescData" function to the VESC library.

    pinMode(13, OUTPUT);

}

//Timing counters
unsigned long tLastVESC_write = 0;
unsigned long tLastVESC_valueRead = 0;
unsigned long tLastHMI_update = 0;
unsigned long tLastSC_update = 0;

void loop() {
    auto start = micros(); //Capture the starting time in microseconds
    ct = millis(); //Capture the system time in milliseconds.  This can be used by other code

    //Begin phased loop code here
    if (ct - tLastVESC_write >= kVESC_WRITE_RATE_MS) {
        //VESC Write
        updateSpeed();
        if (cSC_estopActive) {
            stopAndResetSpeed(); //Reset the speed if the e-stop is active
        }
        VESC_write();
        tLastVESC_write = ct;
    }

    if (ct - tLastVESC_valueRead >= kVESC_READ_RATE_MS) {
        //VESC Read
        bldc_interface_get_values();
        tLastVESC_valueRead = ct;
    }

    if (ct - tLastHMI_update >= kHMI_UPDATE_RATE_MS) {
        //HMI Update
        HMI_update();
        tLastHMI_update = ct;
    }

    if (ct - tLastSC_update >= kSC_UPDATE_RATE_MS) {
        //Safety controller update
        SC_update();
        tLastSC_update = ct;
    }


    VESC_processIncomingData(); //Process any new serial data from the VESC
    bldc_interface_uart_run_timer(); //Updates the VESC library timer
    //End phased loop code here

    //Phase the loop so it runs at the desired rate
    long elapsedMicros = micros() - start;
    long remainingTime = kLOOP_RATE_MICROS - elapsedMicros; //The time we need to delay for to hit the loop phase
    if (remainingTime > 0) delayMicroseconds(remainingTime);

    if (elapsedMicros < 1000) {
        digitalWrite(13, HIGH);
    } else {
        digitalWrite(13, LOW);
    }

    Serial.println(elapsedMicros); //TODO debug only, remove for release
}