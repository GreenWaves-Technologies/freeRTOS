/****************************************************************************/
/* FreeRTOS functions includes. */
#include "FreeRTOS_util.h"

/* Demo utlities includes. */

/****************************************************************************/

/* Test task to test FreeRTOS port. */
void vTestHelloWorld( void *parameters );

/* Utilities to control tasks. */
TaskHandle_t tasks[NBTASKS];
uint8_t taskSuspended = 0;

/****************************************************************************/

/* Variables used. */

/****************************************************************************/

/* Program Entry. */
int main( void )
{
    printf("\n\n\t *** Cluster Hello World Test ***\n\n");

    #if configSUPPORT_DYNAMIC_ALLOCATION == 1

    /* Init memory regions to alloc memory. */
    vPortDefineHeapRegions( xHeapRegions );

    BaseType_t xTask;
    TaskHandle_t xHandleDynamic = NULL;

    xTask = xTaskCreate(
        vTestHelloWorld,
        "TestHelloWorld",
        configMINIMAL_STACK_SIZE * 1,
        NULL,
        tskIDLE_PRIORITY + 1,
        &xHandleDynamic
        );
    if( xTask != pdPASS )
    {
        printf("TestHelloWorld is NULL !\n");
        exit(0);
    }
    #endif //configSUPPORT_DYNAMIC_ALLOCATION

    tasks[0] = xHandleDynamic;

    /* Start the kernel.  From here on, only tasks and interrupts will run. */
    printf("\nScheduler starts !\n");
    vTaskStartScheduler();

    /* Exit FreeRTOS */
    return 0;
}
/*-----------------------------------------------------------*/

void vTestHelloWorld( void *parameters )
{
    ( void ) parameters;
    char *taskname = pcTaskGetName( NULL );
    uint32_t coreid = __core_ID(), clusterid = __cluster_ID();

    printf("Hello World !\n");
    printf("%s on Core %d of cluster %d leaving...\n", taskname, coreid, clusterid);
    vTaskSuspend( NULL );
}
/*-----------------------------------------------------------*/
