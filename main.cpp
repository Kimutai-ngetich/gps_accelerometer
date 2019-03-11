#include "mbed.h"
#include "mbed_trace.h"
#include "mbed_events.h"
#include "LoRaWANInterface.h"
#include "SX1276_LoRaRadio.h"
#include "CayenneLPP.h"
#include "lora_radio_helper.h"
#include "standby.h"
#include "GPS.h"
#include "ADXL345_I2C.h"


#define GPSBaud 9600
#define     STANDBY_TIME_S        1 * 60
#define     SENSOR_READ_ATTEMPTS  3
#define     SENSOR_WAIT_TIME      3000 //slow sensor, no more than once per 2 seconds
  


static uint32_t DEVADDR_1 = 0x260118F6;
static uint8_t NWKSKEY_1[] = { 0x90, 0xF9, 0x6E, 0x7E, 0x9C, 0x38, 0x75, 0xD5, 0x59, 0x75, 0x6D, 0x91, 0x62, 0x22, 0x73, 0x1C };
static uint8_t APPSKEY_1[] = { 0x73, 0x0B, 0x09, 0xB7, 0x93, 0xBA, 0x69, 0xEF, 0x69, 0xC4, 0x0E, 0xEF, 0xBD, 0x43, 0x19, 0x61 };


static Serial pc(SERIAL_TX, SERIAL_RX, 9600);
static GPS gps(PA_9, PB_7);
ADXL345_I2C accelerometer(D14, D15);


// The port we're sending and receiving on
#define MBED_CONF_LORA_APP_PORT     15



// EventQueue is required to dispatch events around
static EventQueue ev_queue;

// Constructing Mbed LoRaWANInterface and passing it down the radio object.
static LoRaWANInterface lorawan(radio);

// Application specific callbacks
static lorawan_app_callbacks_t callbacks;

// LoRaWAN stack event handler
static void lora_event_handler(lorawan_event_t event);

// Send a message over LoRaWAN
static void send_message() {
    CayenneLPP payload(50);
      
        if( gps.sample() == 1) {
           
          float latitude = gps.latitude;
           float longitude = gps.longitude;
            float altitude= (gps.utc/100)+465;
            
            
            printf("latitude: %0.4f, longitude: %0.4f, altitude: %f\r\n",latitude,longitude,altitude);
            
           payload.addGPS(2, latitude,longitude,altitude);  
             
            
            wait(1);
            
             } 
             else
             {
                 printf("no gps detected");
                 }
       
       int readings[3] = {0, 0, 0};
    
    printf("\n Animal activity \n");
    
     printf("Device ID is: 0x%02x\n", accelerometer.getDeviceID());
    wait(.001);
    
    if (accelerometer.setPowerControl(0x00)){
         printf("didn't intitialize power control\n"); 
         }
     //Full resolution, +/-16g, 4mg/LSB.
     wait(.001);
     
     if(accelerometer.setDataFormatControl(0x0B)){
        printf("didn't set data format\n");
         }
     wait(.001);
     
     //3.2kHz data rate.
     if(accelerometer.setDataRate(ADXL345_3200HZ)){
        printf("didn't set data rate\n");
            }
     wait(.001);
     
     //Measurement mode.
     
     if(accelerometer.setPowerControl(MeasurementMode)) {
        printf("didn't set the power control to measurement\n"); 
          } 
         wait(0.01);
         
         accelerometer.getOutput(readings);
         

        printf("X-axis= %i, Y-axis= %i, Z-axis= %i\n", (int16_t)readings[0], (int16_t)readings[1], (int16_t)readings[2]);
        
        payload.addAccelerometer(3,  readings[0], readings[1], readings[2]);
        
        wait(2);

       
    
       
       

    if (payload.getSize() > 0) {
      printf("Sending %d bytes\n", payload.getSize());

      int16_t retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, payload.getBuffer(), payload.getSize(), MSG_UNCONFIRMED_FLAG);

    // for some reason send() ret\urns -1... I cannot find out why, the stack returns the right number. I feel that this is some weird Emscripten quirk
      if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - duty cycle violation\n")
            : printf("send() - Error code %d\n", retcode);

        standby(STANDBY_TIME_S);
      }

      printf("%d bytes scheduled for transmission\n", retcode);
    }

    else
      standby(STANDBY_TIME_S);
}

int main() {
    set_time(0);

    printf("\r==========================\n");
    printf("\r  cattle monitor     \n");
    printf("\r==========================\n");

    printf("Sending every %d seconds\n", STANDBY_TIME_S);

    // Enable trace output for this demo, so we can see what the LoRaWAN stack does
    mbed_trace_init();

    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("LoRa initialization failed!\n");
        return -1;
    }

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Disable adaptive data rating
    if (lorawan.disable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\rdisable_adaptive_datarate failed!\n");
        return -1;
    }

    lorawan.set_datarate(0); // SF12BW125

    lorawan_connect_t connect_params;
    connect_params.connect_type = LORAWAN_CONNECTION_ABP;

    connect_params.connection_u.abp.dev_addr = DEVADDR_1;
    connect_params.connection_u.abp.nwk_skey = NWKSKEY_1;
    connect_params.connection_u.abp.app_skey = APPSKEY_1;

    lorawan_status_t retcode = lorawan.connect(connect_params);

    if (retcode == LORAWAN_STATUS_OK ||
        retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("Connection error, code = %d\n", retcode);
        return -1;
    }

    printf("Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();
}

// This is called from RX_DONE, so whenever a message came in
static void receive_message()
{
    uint8_t rx_buffer[50] = { 0 };
    int16_t retcode;
    retcode = lorawan.receive(MBED_CONF_LORA_APP_PORT, rx_buffer,
                              sizeof(rx_buffer),
                              MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        printf("receive() - Error code %d\n", retcode);
        return;
    }

    printf("Data received on port %d (length %d): ", MBED_CONF_LORA_APP_PORT, retcode);

    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\n");
}

// Event handler
static void lora_event_handler(lorawan_event_t event) {
    switch (event) {
        case CONNECTED:
            printf("Connection - Successful\n");
            ev_queue.call_in(1000, &send_message);
            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("Disconnected Successfully\n");
            break;
        case TX_DONE:
            printf("Message Sent to Network Server\n");
            standby(STANDBY_TIME_S);
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("Transmission Error - EventCode = %d\n", event);
            standby(STANDBY_TIME_S);
            break;
        case RX_DONE:
            printf("Received message from Network Server\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("Error in reception - Code = %d\n", event);
            break;
        case JOIN_FAILURE:
            printf("OTAA Failed - Check Keys\n");
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}
