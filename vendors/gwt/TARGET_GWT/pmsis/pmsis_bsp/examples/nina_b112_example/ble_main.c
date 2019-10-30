/* PMSIS includes. */
#include "pmsis.h"
#include "bsp/gapoc_a.h"
#include "bsp/ble/nina_b112/nina_b112.h"

/* App includes. */
#include "ble_protocol.h"
#include <fcntl.h>
#include "ImgIO.h"

#define QUEUE_SIZE 4
#define AT_RESP_ARRAY_LENGTH 64

#define MIN(a, b) (((a)<(b))?(a):(b))

unsigned char face_image_buffer[QUEUE_SIZE][128*128];
short face_descriptor[QUEUE_SIZE][512];
char name[QUEUE_SIZE][16] = {"Lena", "Francesco"};

unsigned int queue_head = 0;
unsigned int queue_tail = 2;

char string_buffer[127];
char current_name[16];
short current_descriptor[512];

pi_nina_ble_t ble;
pi_task_t name_task;
pi_task_t teskriptor_task;

char empty_response = '\0';
unsigned char ack = BLE_ACK;
unsigned char action = 0;
char* ptr = NULL;

static uint8_t rx_buffer[AT_RESP_ARRAY_LENGTH];

void body(void* parameters)
{
    char Resp_String[AT_RESP_ARRAY_LENGTH];

    unsigned int width = 128, height = 128;

    BRIDGE_Init();
    printf("BRIDGE_Init done\n");

    BRIDGE_Connect(0, NULL);
    printf("BRIDGE_Connect done\n");

    for(int i = 0; i < 2; i++)
    {
        printf("Reading input image...\n");
        sprintf(string_buffer, "../../../%s.pgm", name[i]);
        int bridge_status = (int) ReadImageFromFile(string_buffer, &width, &height, face_image_buffer[i], width*height*sizeof(char));
        if(bridge_status == 0)
        {
            printf("Face image load failed\n");
            pmsis_exit(0);
        }

        printf("Reading input image...done\n");

        sprintf(string_buffer, "../../../%s.bin", name[i]);
        int descriptor_file = BRIDGE_Open(string_buffer, 0, 0, NULL);
        if(descriptor_file < 0)
        {
            printf("Face descriptor open failed\n");
            pmsis_exit(0);
        }

        bridge_status = BRIDGE_Read(descriptor_file, face_descriptor[i], 512*sizeof(short), NULL);

        if(bridge_status != 512*sizeof(short))
        {
            printf("Face descriptor read failed\n");
            pmsis_exit(0);
        }

        BRIDGE_Close(descriptor_file, NULL);
    }

    pi_pad_set_function(CONFIG_HYPERBUS_DATA6_PAD, CONFIG_UART_RX_PAD_FUNC);

    pi_nina_b112_open(&ble);
    printf("BLE UART init done\n");

    // Init GPIO that will control NINA DSR in deasserted position
    pi_gpio_pin_write(0, GPIOA21_NINA17, 0);

    // Enable BLE (release reset)
    pi_gpio_pin_write(0, GPIOA2_NINA_RST, 1);

    pi_time_wait_us(1*1000*1000);

    // Initiliaze NINA as BLE Peripheral

    printf("Sending cmd using pmsis bsp\n");
    pi_nina_b112_AT_send(&ble, "E0");
    printf("Echo disabled\n");
    pi_nina_b112_AT_send(&ble, "+UFACTORY");
    printf("Factory configuration restored\n");
    pi_nina_b112_AT_send(&ble, "+UBTUB=FFFFFFFFFFFF");
    printf("Set UBTUB\n");
    pi_nina_b112_AT_send(&ble, "+UBTLE=2");
    printf("Set UBTLE\n");
    pi_nina_b112_AT_send(&ble, "+UBTLN=GreenWaves-GAPOC");
    printf("Set UBTLN\n");
    pi_nina_b112_AT_query(&ble, "+UMRS?", (char *) rx_buffer);
    printf("BLE configuration : %s\n", rx_buffer);
    pi_nina_b112_AT_query(&ble, "+UBTLN?", (char *) rx_buffer);
    printf("BLE name : %s\n", rx_buffer);
    //nina_b112_close(&ble);

    printf("AT Config Done\n");

    // After Reboot of NINA,  central connects to NINA and NINA will provide
    // unsollicited AT event: +UUBTACLC:<peer handle,0,<remote BT address>)
    // (...but sometimes just provides empty event instead !?)

    // Just make sure NINA sends something as AT unsolicited response, therefore is ready :
    pi_nina_b112_wait_for_event(&ble, Resp_String);
    printf("Received Event after reboot: %s\n", Resp_String);

    // Enter Data Mode
    pi_nina_b112_AT_send(&ble, "O");
    printf("Data Mode Entered!\n");

    pi_time_wait_us(1*1000*1000);

    char read_mode  = 0;
    char write_mode = 0;
    int face_chank_idx = 0;
    int descriptor_chank_idx = 0;
    uint8_t ble_exit = 0;

    while(!ble_exit)
    {
        pi_nina_b112_get_data_blocking(&ble, &action, 1);

        switch(action)
        {
            case BLE_READ:
                printf("BLE_READ request got\n");
                printf("Queue head: %d, Queue tail: %d\n", queue_head, queue_tail);
                if(queue_head != queue_tail)
                {
                    // there is something in queue
                    read_mode = 1;
                    pi_nina_b112_send_data_blocking(&ble, &ack, 1);
                    printf("BLE_ACK responded\n");
                }
                else
                {
                    printf("Nothing to read\n");
                    pi_nina_b112_send_data_blocking(&ble, (uint8_t *) &empty_response, 1);
                }
                break;
            case BLE_GET_NAME:
                printf("BLE_GET_NAME request got\n");
                if(read_mode && (queue_head != queue_tail)) // we are reading and have something in queue
                {
                    pi_nina_b112_send_data_blocking(&ble, (uint8_t *) name[queue_head], strlen(name[queue_head]));
                    printf("Name %s responded\n", name[queue_head]);
                }
                else
                {
                    printf("ERROR: Empty respond sent\n");
                    pi_nina_b112_send_data_blocking(&ble, (uint8_t *) &empty_response, 1);
                }
                break;
            case BLE_GET_PHOTO:
                printf("BLE_GET_PHOTO request got\n");
                if(read_mode && (queue_head != queue_tail)) // we are reading and have something in queue
                {

                    ptr = (char *) (face_image_buffer[queue_head] + face_chank_idx * DATA_CHANK_SIZE);
                    int size = MIN(DATA_CHANK_SIZE, 128*128-face_chank_idx * DATA_CHANK_SIZE);
                    pi_nina_b112_send_data_blocking(&ble,(uint8_t *) ptr, size);
                    face_chank_idx++;
                    int iters = (128*128 + DATA_CHANK_SIZE-1) / DATA_CHANK_SIZE;
                    if(face_chank_idx >= iters)
                    {
                        face_chank_idx = 0;
                        printf("Face photo transfer finished\n");
                    }
                    printf("Face photo sent (%d bytes)\n", size);
                }
                else
                {
                    printf("ERROR: Empty respond sent\n");
                    pi_nina_b112_send_data_blocking(&ble,(uint8_t *) &empty_response, 1);
                }
                break;
            case BLE_GET_DESCRIPTOR:
                printf("BLE_GET_DESCRIPTOR request got\n");
                if(read_mode && (queue_head != queue_tail)) // we are reading and have something in queue
                {
                    pi_nina_b112_send_data_blocking(&ble,(uint8_t *) face_descriptor[queue_head], 512*sizeof(short));
                    printf("Face descriptor sent\n");
                }
                else
                {
                    printf("ERROR: Empty respond sent\n");
                    pi_nina_b112_send_data_blocking(&ble,(uint8_t *) &empty_response, 1);
                }
                break;
            case BLE_REMOVE:
                printf("BLE_REMOVE request got\n");
                if(read_mode && (queue_head != queue_tail))
                {
                    read_mode = 0;
                    queue_head++;
                    if(queue_head == QUEUE_SIZE)
                    {
                        queue_head = 0;
                    }
                    printf("Queue head: %d, Queue tail: %d\n", queue_head, queue_tail);
                    pi_nina_b112_send_data_blocking(&ble,(uint8_t *) &ack, 1);
                    printf("BLE_ACK responded\n");
                }
                else
                {
                    pi_nina_b112_send_data_blocking(&ble,(uint8_t *) &empty_response, 1);
                    printf("ERROR: Empty respond sent\n");
                }
                break;

            case BLE_WRITE:
                printf("BLE_WRITE request got\n");
                write_mode = 1;
                pi_nina_b112_send_data_blocking(&ble,(uint8_t *) &ack, 1);
                printf("BLE_ACK responded\n");
                break;
            case BLE_SET_NAME:
                pi_nina_b112_get_data_blocking(&ble, (uint8_t *) current_name, 16);
                current_name[15] = '\0';
                printf("Name %s got\n", current_name);

                pi_nina_b112_send_data_blocking(&ble,(uint8_t *)  &ack, 1);
                printf("BLE_ACK responded\n");
                if (write_mode && (queue_head != queue_tail))
                {
                    strcpy(name[queue_head], current_name);
                }
                break;
            case BLE_SET_DESCRIPTOR:
            {
                int chunk_size = 10;
                int packages = 512 / chunk_size;
                for (int i = 0; i < packages; i++)
                {
                    int start = i*chunk_size;
                    pi_nina_b112_get_data_blocking(&ble, (uint8_t *) current_descriptor+start, chunk_size*sizeof(short));
                }
                pi_nina_b112_get_data_blocking(&ble,(uint8_t *) current_descriptor+packages*chunk_size, (512-packages*chunk_size)*sizeof(short));
                printf("BLE_SET_DESCRIPTOR request got\n");
                printf("Got face descriptor\n");

                sprintf(string_buffer, "../../../%s.bin", current_name);
                printf("Writing descriptor file \"%s\" ..\n", string_buffer);

                int descriptor_file = BRIDGE_Open(string_buffer, O_RDWR | O_CREAT, S_IRWXU, NULL);
                if(descriptor_file < 0)
                {
                    printf("Face descriptor open failed\n");
                    pmsis_exit(0);
                }

                int bridge_status = BRIDGE_Write(descriptor_file, current_descriptor, 512*sizeof(short), NULL);

                if(bridge_status != 512*sizeof(short))
                {
                    printf("Face descriptor read failed\n");
                    pmsis_exit(0);
                }

                BRIDGE_Close(descriptor_file, NULL);
                printf("Writing descriptor file..done\n");


                pi_nina_b112_send_data_blocking(&ble,(uint8_t *)  &ack, 1);
                printf("BLE_ACK responded\n");
                // Add to Known People DB here
            } break;

            case BLE_EXIT:
                printf("BLE_EXIT request got\n");
                printf("Closing BLE connection\n");
                pi_nina_b112_close(&ble);
                ble_exit = 1;
                break;
            default:
                printf("Error: invalid request %d\n", action);
        }

    }

    printf("Exiting BLE test\n");

    BRIDGE_Disconnect(NULL);

    pmsis_exit(0);
}

int main(void)
{
    printf("Start NINA BLE module test\n");
    return pmsis_kickoff((void *) body);
}
