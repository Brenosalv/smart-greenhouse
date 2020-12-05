/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* Standard includes. */
#include "stdio.h"
#include "stdlib.h"
#include <time.h>

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include <queue.h>
#include <semphr.h>

/* The embedded system to be developed contains two sensing modules that collect information from
temperature and brightness of an environment. This same system performs two procedures for processing
independent data, which uses temperature and luminosity information to identify whether the system
monitored is in an ideal condition. The monitored environment is an automated and intelligent greenhouse,
composed of a mechanical arrangement (greenhouse) and an intelligent embedded system with connectivity, which will perform the
control and automation of vegetable cultivation inside the greenhouse. */

/* This project uses heap_5.c, and these constants define the sizes of the regions
that make up the total heap.  heap_5 is only used for test and example purposes
as this demo could easily create one large heap region instead of multiple
smaller heap regions - in which case heap_4.c would be the more appropriate
choice.  See http://www.freertos.org/a00111.html for an explanation. */
#define mainREGION_1_SIZE	7201
#define mainREGION_2_SIZE	29905
#define mainREGION_3_SIZE	6407
#define SIZE 10
#define ROW SIZE
#define COL SIZE

xTaskHandle priority_handle = NULL;
xTaskHandle matrix_handle;
xTaskHandle communication_handle;
xTaskHandle Mod_T_handle;
xTaskHandle Mod_L_handle;
xTaskHandle alarmTask1_handle;
xTaskHandle alarmTask2_handle;
xQueueHandle  Buffer_T;
xQueueHandle  Buffer_L;
SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSemaphore_2;

long matrix_ticks = 0;
long comm_ticks = 0;
long total_run = 0;
long ticks = 0;

  /*
   * This demo uses heap_5.c, so start by defining some heap regions.  It is not
   * necessary for this demo to use heap_5, as it could define one large heap
   * region.  Heap_5 is only used for test and example purposes.  See
   * http://www.freertos.org/a00111.html for an explanation.
   */
static void  prvInitialiseHeap(void);

/*
 * Prototypes for the standard FreeRTOS application hook (callback) functions
 * implemented within this file.  See http://www.freertos.org/a00016.html .
 */
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char* pcTaskName);
void vApplicationTickHook(void);
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize);
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer, StackType_t** ppxTimerTaskStackBuffer, uint32_t* pulTimerTaskStackSize);

/*
 * Writes trace data to a disk file when the trace recording is stopped.
 * This function will simply overwrite any trace files that already exist.
 */
static void prvSaveTraceFile(void);

/*-----------------------------------------------------------*/

/* When configSUPPORT_STATIC_ALLOCATION is set to 1 the application writer can
use a callback function to optionally provide the memory required by the idle
and timer tasks.  This is the stack that will be used by the timer task.  It is
declared here, as a global, so it can be checked by a test that is implemented
in a different file. */
StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

/* Notes if the trace is running or not. */
static BaseType_t xTraceRunning = pdTRUE;

/*-----------------------------------------------------------*/
// Task used to alloc memory and process data
static void matrix_task()
{
	int i;
	double** a = (double**)pvPortMalloc(ROW * sizeof(double*));
	for (i = 0; i < ROW; i++) a[i] = (double*)pvPortMalloc(COL * sizeof(double));
	double** b = (double**)pvPortMalloc(ROW * sizeof(double*));
	for (i = 0; i < ROW; i++) b[i] = (double*)pvPortMalloc(COL * sizeof(double));
	double** c = (double**)pvPortMalloc(ROW * sizeof(double*));
	for (i = 0; i < ROW; i++) c[i] = (double*)pvPortMalloc(COL * sizeof(double));

	double sum = 0.0;
	int j, k, l;

	for (i = 0; i < SIZE; i++) {
		for (j = 0; j < SIZE; j++) {
			a[i][j] = 1.5;
			b[i][j] = 2.6;
		}
	}

	while (1) {
		/*
		* In an embedded systems, matrix multiplication would block the CPU for a long time
		* but since this is a PC simulator we must add one additional dummy delay.
		*/
		printf("Starting Processing data...\n");
		 
		long simulationdelay;
		for (simulationdelay = 0; simulationdelay < 1000000000; simulationdelay++)
			;
		for (i = 0; i < SIZE; i++) {
			for (j = 0; j < SIZE; j++) {
				c[i][j] = 0.0;
			}
		}

		for (i = 0; i < SIZE; i++) {
			for (j = 0; j < SIZE; j++) {
				sum = 0.0;
				for (k = 0; k < SIZE; k++) {
					for (l = 0; l < 10; l++) {
						sum = sum + a[i][k] * b[k][j];
					}
				}
				c[i][j] = sum;
			}
		}
		vTaskDelay(100);
		printf("Ending Processing data...\n");
	}
	
	// Deallocation of memory resources
	for (i = 0; i < SIZE; i++) {
		vPortFree(a[i]);
		vPortFree(b[i]);
		vPortFree(c[i]);
	}

	vPortFree(a);
	vPortFree(b);
	vPortFree(c);
}

// Function used to invert alarmTask1 and alarmTask2 priorities when alarmTask2 runs 100 times
static void prioritySetTask(int cont)
{
	while (1)
	{
		if (cont == 100)
		{
			vTaskPrioritySet(alarmTask2_handle, alarmTask1_handle);
			printf("Prioridades das tarefas alarmTask1 e alarmTask2 invertidas.\n");
		}
		vTaskDelay(10);
		break;
	}
}

// Task used to send data to cloud every 200ms
static void communication_task()
{
	while (1) {
		printf("Sending data...\n");
		 
		vTaskDelay(100);
		printf("Data sent!\n");
		 
		vTaskDelay(100);
	}
}

// Declaração das variáveis temperatura e luminosidade.
float  temperatura ;
float luminosidade ;

// Tarefa que tem o papel de fazer o sensoriamento da temperatura a cada 800ms.
static void Mod_T() 
{
	while (1) {
		// Simulando uma temperatura aleatória entre 5°C e 30°C, a qual é lida pelo sensor de temperatura.
			 temperatura = 5 + (rand() % 2500) / 100;

			// Enviando dados de temperatura para a fila Buffer_T.
			if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
				xQueueSendToBack(Buffer_T, & temperatura, pdMS_TO_TICKS(0));
				printf("Lendo temperatura do sensor: %.2f Graus Celsius\n",  temperatura);
				taskYIELD();
				xSemaphoreGive(xSemaphore);
			}
		vTaskDelay(800);
	}
}

// Tarefa que tem o papel de fazer o sensoriamento da luminosidade a cada 1600ms.
static void Mod_L()
{
	while (1) {
		// Simulando uma luminosidade aleatória entre 7000 lumens e 8000 lumens, a qual é lida pelo sensor de luminosidade.
		luminosidade = 7000 + (rand() % 10000) / 10;

		// Enviando dados de luminosidade para a fila Buffer_L.
		if (xSemaphoreTake(xSemaphore_2, (TickType_t)10) == pdTRUE) {
			xQueueSendToBack(Buffer_L, &luminosidade, pdMS_TO_TICKS(0));
			printf("Lendo luminosidade do sensor: %.2f Lumens\n", luminosidade);
			taskYIELD();
			xSemaphoreGive(xSemaphore_2);
		}
		vTaskDelay(1600);
	}
}

// Considera-se que a temperatura ideal para uma estufa é em média de 10°C a 20°C.
// Esta tarefa tem o papel de soar o alarme caso a temperatura da estuda esteja excessiva, ou seja, se a temperatura exceder 20°C:
static void alarmTask1()
{
	while (1) {
		if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
			if (xQueuePeek(Buffer_T, & temperatura, pdMS_TO_TICKS(0)) == pdPASS) {
				if ( temperatura > 20)
					printf("Alarme soado! A temperatura atual deve ser regulada. Temperatura atual da estufa = %.2f\n",  temperatura);
				else
					printf("A temperatura atual e apropriada para a estufa. Temperatura atual da estufa = %.2f.\n",  temperatura);
			}
			xSemaphoreGive(xSemaphore);
		}
			taskYIELD();
			vTaskDelay(1600);
	}
}

/* Atualmente, pesquisas dão conta de que o mínimo de luz necessária para sustentar o crescimento
é de 2.000 lumens por pé quadrado. A média seria de 5.000 lumens por pé quadrado e, o ideal, de 7.000
a 7.500 por pé quadrado. */
/* Esta tarefa tem o papel de soar o alarme caso a luminosidade esteja alta em relação
à temperatura excessiva da estufa, ou seja, se a luminosidade exceder 7500 lux: */
static void alarmTask2()
{
	int contador = 0;

	while (1) {
		if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE && xSemaphoreTake(xSemaphore_2, (TickType_t)10) == pdTRUE) {
			if (xQueueReceive(Buffer_T, & temperatura, pdMS_TO_TICKS(0)) == pdPASS && xQueueReceive(Buffer_L, &luminosidade, pdMS_TO_TICKS(0)) == pdPASS) {
				if ( temperatura > 20 && luminosidade > 7500)
					printf("Alarme soado! A temperatura e luminosidade atuais devem ser reguladas. Temperatura atual = %.2f; Luminosidade atual = %.2f\n",  temperatura, luminosidade);
				else
					printf("A temperatura e a luminosidade atuais sao apropriadas para a estufa. Temperatura atual = %.2f; Luminosidade atual = %.2f\n",  temperatura, luminosidade);
			}
			xSemaphoreGive(xSemaphore);
			xSemaphoreGive(xSemaphore_2);
		}
			taskYIELD();
			vTaskDelay(1600);
			 
			// Sempre que a tarefa alarmTask2 for executada, o contador soma + 1.
			contador++;

			/* A função de mudança de prioridade é chamada para verificar se a tarefa alarmTask2 foi executada 100
			vezes para que a prioridade seja invertida com a tarefa alarmTask1 */
			prioritySetTask(contador);
	}
}

// The main function is used to create queues and semaphores as much as create tasks and initialize the scheduler
int main(void)
{
	/* This demo uses heap_5.c, so start by defining some heap regions.  heap_5
	is only used for test and example reasons.  Heap_4 is more appropriate.  See
	http://www.freertos.org/a00111.html for an explanation. */
	prvInitialiseHeap();

	/* Initialise the trace recorder.  Use of the trace recorder is optional.
	See http://www.FreeRTOS.org/trace for more information. */
	vTraceEnable(TRC_START);
	
	// Criação das filas e semáforos que são usados para controle de sincronia das tarefas e exclusão mútua.
	Buffer_T = xQueueCreate(1, sizeof( temperatura));
	Buffer_L = xQueueCreate(1, sizeof(luminosidade));
	xSemaphore = xSemaphoreCreateMutex();
	xSemaphore_2 = xSemaphoreCreateMutex();

	// Criando tarefas
	xTaskCreate(matrix_task, (signed char*)"Matrix", 1000, (void*)NULL, 3, &matrix_handle);
	xTaskCreate(communication_task, (signed char*)"Communication", configMINIMAL_STACK_SIZE, (void*)NULL, 1, &communication_handle);
	xTaskCreate(Mod_L, (signed char*)"Mod_L", configMINIMAL_STACK_SIZE, (void*)NULL, 2, &Mod_L_handle);
	xTaskCreate(Mod_T, (signed char*)"Mod_T", configMINIMAL_STACK_SIZE, (void*)NULL, 2, &Mod_T_handle);
	xTaskCreate(alarmTask1, (signed char*)"alarmTask1", configMINIMAL_STACK_SIZE, (void*)NULL, 2, &alarmTask1_handle);
	xTaskCreate(alarmTask2, (signed char*)"alarmTask2", configMINIMAL_STACK_SIZE, (void*)NULL,2, &alarmTask2_handle);
	// Implementando o escalonador
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c, heap_2.c or heap_4.c is being used, then the
	size of the	heap available to pvPortMalloc() is defined by
	configTOTAL_HEAP_SIZE in FreeRTOSConfig.h, and the xPortGetFreeHeapSize()
	API function can be used to query the size of free heap space that remains
	(although it does not provide information on how the remaining heap might be
	fragmented).  See http://www.freertos.org/a00111.html for more
	information. */
	vAssertCalled(__LINE__, __FILE__);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If application tasks make use of the
	vTaskDelete() API function to delete themselves then it is also important
	that vApplicationIdleHook() is permitted to return to its calling function,
	because it is the responsibility of the idle task to clean up memory
	allocated by the kernel to any task that has since deleted itself. */

	/* Uncomment the following code to allow the trace to be stopped with any
	key press.  The code is commented out by default as the kbhit() function
	interferes with the run time behaviour. */
	/*
		if( _kbhit() != pdFALSE )
		{
			if( xTraceRunning == pdTRUE )
			{
				vTraceStop();
				prvSaveTraceFile();
				xTraceRunning = pdFALSE;
			}
		}
	*/

#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
	{
		/* Call the idle task processing used by the full demo.  The simple
		blinky demo does not use the idle task hook. */
		vFullDemoIdleFunction();
	}
#endif
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char* pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  This function is
	provided as an example only as stack overflow checking does not function
	when running the FreeRTOS Windows port. */
	vAssertCalled(__LINE__, __FILE__);
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
	{
		vFullDemoTickHookFunction();
	}
#endif /* mainCREATE_SIMPLE_BLINKY_DEMO_ONLY */
}
/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook(void)
{
	/* This function will be called once only, when the daemon task starts to
	execute	(sometimes called the timer task).  This is useful if the
	application includes initialisation code that would benefit from executing
	after the scheduler has been started. */
}
/*-----------------------------------------------------------*/

void vAssertCalled(unsigned long ulLine, const char* const pcFileName)
{
	static BaseType_t xPrinted = pdFALSE;
	volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

	/* Called if an assertion passed to configASSERT() fails.  See
	http://www.freertos.org/a00110.html#configASSERT for more information. */

	/* Parameters are not used. */
	(void)ulLine;
	(void)pcFileName;

	printf("ASSERT! Line %ld, file %s, GetLastError() %ld\r\n", ulLine, pcFileName, GetLastError());

	taskENTER_CRITICAL();
	{
		/* Stop the trace recording. */
		if (xPrinted == pdFALSE)
		{
			xPrinted = pdTRUE;
			if (xTraceRunning == pdTRUE)
			{
				vTraceStop();
				prvSaveTraceFile();
			}
		}

		/* You can step out of this function to debug the assertion by using
		the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
		value. */
		while (ulSetToNonZeroInDebuggerToContinue == 0)
		{
			__asm { NOP };
			__asm { NOP };
		}
	}
	taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

static void prvSaveTraceFile(void)
{
	FILE* pxOutputFile;

	fopen_s(&pxOutputFile, "Trace.dump", "wb");

	if (pxOutputFile != NULL)
	{
		fwrite(RecorderDataPtr, sizeof(RecorderDataType), 1, pxOutputFile);
		fclose(pxOutputFile);
		printf("\r\nTrace output saved to Trace.dump\r\n");
	}
	else
	{
		printf("\r\nFailed to create trace dump file\r\n");
	}
}
/*-----------------------------------------------------------*/

static void  prvInitialiseHeap(void)
{
	/* The Windows demo could create one large heap region, in which case it would
	be appropriate to use heap_4.  However, purely for demonstration purposes,
	heap_5 is used instead, so start by defining some heap regions.  No
	initialisation is required when any other heap implementation is used.  See
	http://www.freertos.org/a00111.html for more information.

	The xHeapRegions structure requires the regions to be defined in start address
	order, so this just creates one big array, then populates the structure with
	offsets into the array - with gaps in between and messy alignment just for test
	purposes. */
	static uint8_t ucHeap[configTOTAL_HEAP_SIZE];
	volatile uint32_t ulAdditionalOffset = 19; /* Just to prevent 'condition is always true' warnings in configASSERT(). */
	const HeapRegion_t xHeapRegions[] =
	{
		/* Start address with dummy offsets						Size */
		{ ucHeap + 1,											mainREGION_1_SIZE },
		{ ucHeap + 15 + mainREGION_1_SIZE,						mainREGION_2_SIZE },
		{ ucHeap + 19 + mainREGION_1_SIZE + mainREGION_2_SIZE,	mainREGION_3_SIZE },
		{ NULL, 0 }
	};

	/* Sanity check that the sizes and offsets defined actually fit into the
	array. */
	configASSERT((ulAdditionalOffset + mainREGION_1_SIZE + mainREGION_2_SIZE + mainREGION_3_SIZE) < configTOTAL_HEAP_SIZE);

	/* Prevent compiler warnings when configASSERT() is not defined. */
	(void)ulAdditionalOffset;

	vPortDefineHeapRegions(xHeapRegions);
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize)
{
	/* If the buffers to be provided to the Idle task are declared inside this
	function then they must be declared static - otherwise they will be allocated on
	the stack and so not exists after this function exits. */
	static StaticTask_t xIdleTaskTCB;
	static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

	/* Pass out a pointer to the StaticTask_t structure in which the Idle task's
	state will be stored. */
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

	/* Pass out the array that will be used as the Idle task's stack. */
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;

	/* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer, StackType_t** ppxTimerTaskStackBuffer, uint32_t* pulTimerTaskStackSize)
{
	/* If the buffers to be provided to the Timer task are declared inside this
	function then they must be declared static - otherwise they will be allocated on
	the stack and so not exists after this function exits. */
	static StaticTask_t xTimerTaskTCB;

	/* Pass out a pointer to the StaticTask_t structure in which the Timer
	task's state will be stored. */
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

	/* Pass out the array that will be used as the Timer task's stack. */
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;

	/* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

