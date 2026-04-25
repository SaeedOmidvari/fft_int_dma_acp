#include <stdio.h>
#include <stdlib.h>
#include <complex.h>
#include <xtime_l.h>   // Timer for execution time calculations
#include "xaxidma.h"
#include "xparameters.h"
#include "platform.h"
#include "xscugic.h"


#define FFT_Size 8
#define EXPT 30

//#define MEM_BASE_ADDR      (DDR_BASE_ADDR + 0x1000000)
//#define TX_BUFFER_BASE     (MEM_BASE_ADDR + 0x00100000)
//#define RX_BUFFER_BASE     (MEM_BASE_ADDR + 0x00300000)

#define RESET_TIMEOUT_COUNTER 10000

XScuGic INTCInst;

/*
 * Flags interrupt handlers use to notify the application context the events.
 */
volatile int MM2SDone;
volatile int S2MMDone;
volatile int Error;


const float complex twiddle_factors[FFT_Size/2] = {
    1 - 0*I,
    0.7071067811865476 - 0.7071067811865475*I,
    0.0 - 1*I,
   -0.7071067811865475 - 0.7071067811865476*I
};

// This function reorders the input to get the output in the normal order
// Refer the handout for the desired input order
const int input_reorder[FFT_Size] = {0, 4, 2, 6, 1, 5, 3, 7};

void InputReorder(float complex dataIn[FFT_Size], float complex dataOut[FFT_Size])
{
    for (int i = 0; i < FFT_Size; i++)
    {
        dataOut[i] = dataIn[input_reorder[i]];
    }
}


// For FFT of size FFT_Size, the number of butterfly stages are 2^stages = FFT_Size.
// For 8-point FFT, there are three butterfly stages.

void FFTStages(float complex FFT_input[FFT_Size], float complex FFT_output[FFT_Size])
{
    float complex stage1_out[FFT_Size], stage2_out[FFT_Size];

    // Stage 1
    for (int i = 0; i < FFT_Size; i = i + 2)
    {
        stage1_out[i]     = FFT_input[i] + FFT_input[i + 1];
        stage1_out[i + 1] = FFT_input[i] - FFT_input[i + 1];
    }

    // Stage 2
    for (int i = 0; i < FFT_Size; i = i + 4)
    {
        for (int j = 0; j < 2; ++j)
        {
            stage2_out[i + j]     = stage1_out[i + j] + twiddle_factors[2 * j] * stage1_out[i + j + 2];
            stage2_out[i + 2 + j] = stage1_out[i + j] - twiddle_factors[2 * j] * stage1_out[i + j + 2];
        }
    }

    // Stage 3
    for (int i = 0; i < FFT_Size / 2; i++)
    {
        FFT_output[i]     = stage2_out[i] + twiddle_factors[i] * stage2_out[i + 4];
        FFT_output[i + 4] = stage2_out[i] - twiddle_factors[i] * stage2_out[i + 4];
    }
}

static void MM2SIntrHandler(void *Callback)
{
    u32 IrqStatus;
    int TimeOut;
    XAxiDma *AxiDmaInst = (XAxiDma *)Callback;

    /* Read pending interrupts */
    IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DEVICE_TO_DMA);

    /* Acknowledge pending interrupts */
    XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DEVICE_TO_DMA);

    // Check whether correct DMA has raised the interrupt
    if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK))
    {
        return;
    }

    /*
     * If error interrupt is asserted, raise error flag, reset the
     * hardware to recover from the error, and return with no further
     * processing.
     */
    if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK))
    {
        Error = 1;

        /*
         * Reset could fail and hang
         * NEED a way to handle this or do not call it??
         */
        XAxiDma_Reset(AxiDmaInst);

        TimeOut = RESET_TIMEOUT_COUNTER;
        while (TimeOut)
        {
            if (XAxiDma_ResetIsDone(AxiDmaInst))
            {
                break;
            }

            TimeOut -= 1;
        }

        return;
    }

    // If completion interrupt is asserted, then set S2MMDone flag
    if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK))
    {
        MM2SDone = 1;
    }
}

static void S2MMIntrHandler(void *Callback)
{
    u32 IrqStatus;
    int TimeOut;
    XAxiDma *AxiDmaInst = (XAxiDma *)Callback;

    /* Read pending interrupts */
    IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DEVICE_TO_DMA);

    /* Acknowledge pending interrupts */
    XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DEVICE_TO_DMA);

    // Check whether correct DMA has raised the interrupt
    if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK))
    {
        return;
    }

    /*
     * If error interrupt is asserted, raise error flag, reset the
     * hardware to recover from the error, and return with no further
     * processing.
     */
    if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK))
    {
        Error = 1;

        /*
         * Reset could fail and hang
         * NEED a way to handle this or do not call it??
         */
        XAxiDma_Reset(AxiDmaInst);

        TimeOut = RESET_TIMEOUT_COUNTER;
        while (TimeOut)
        {
            if (XAxiDma_ResetIsDone(AxiDmaInst))
            {
                break;
            }

            TimeOut -= 1;
        }

        return;
    }

    // If completion interrupt is asserted, then set S2MMDone flag
    if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK))
    {
        S2MMDone = 1;
    }
}

static int SetupIntrSystem(XScuGic *IntcInstancePtr,
                           XAxiDma *AxiDmaPtr,
                           u16 MM2SIntrId,
                           u16 S2MMIntrId)
{
    int Status;
    XScuGic_Config *IntcConfig;

    IntcConfig = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);
    if (NULL == IntcConfig)
    {
        return XST_FAILURE;
    }

    Status = XScuGic_CfgInitialize(IntcInstancePtr,
                                   IntcConfig,
                                   IntcConfig->CpuBaseAddress);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // Initialize Exception handling on the ARM processor
    Xil_ExceptionInit();

    // Connect the supplied Xilinx general interrupt handler
    // to the interrupt handling logic in the processor.
    // All interrupts go through the interrupt controller, so the
    // ARM processor has to first "ask" the interrupt controller
    // which peripheral generated the interrupt. The handler that
    // does this is supplied by Xilinx and is called "XScuGic_InterruptHandler"
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                                 (void *)IntcInstancePtr);
    /*
     * Connect the device driver handler that will be called when an
     * interrupt for the device occurs, the handler defined above performs
     * the specific interrupt processing for the device.
     */

    // Assign/connect our interrupt handler
    Status = XScuGic_Connect(IntcInstancePtr, MM2SIntrId,
                             (Xil_InterruptHandler)MM2SIntrHandler, AxiDmaPtr);
    if (Status != XST_SUCCESS) {
        return Status;
    }

    Status = XScuGic_Connect(IntcInstancePtr, S2MMIntrId,
                             (Xil_InterruptHandler)S2MMIntrHandler, AxiDmaPtr);
    if (Status != XST_SUCCESS) {
        return Status;
    }

    // Enable the interrupt input on the GIC for the DMA interrupt
    XScuGic_Enable(IntcInstancePtr, MM2SIntrId);
    XScuGic_Enable(IntcInstancePtr, S2MMIntrId);

    XScuGic_SetPriorityTriggerType(IntcInstancePtr, MM2SIntrId, 0xA0, 0x3);
    XScuGic_SetPriorityTriggerType(IntcInstancePtr, S2MMIntrId, 0xA0, 0x3);

    /* Enable all interrupts */
    XAxiDma_IntrEnable(AxiDmaPtr, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    XAxiDma_IntrEnable(AxiDmaPtr, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

    // Enable interrupts in the ARM Processor.
    Xil_ExceptionEnable();

    return XST_SUCCESS;
}

int FFTPSvsACP_intr()
{
    int status;

    // For FFT_Size point FFT, define the input.
    // You may modify the code to take the input from user via UART
    float complex FFT_input[FFT_Size];   // = {11+23*I,32+10*I,91+94*I,15+69*I,47+96*I,44+12*I,96+17*I,49+58*I};

    // FFT output will be stored in this variable
    float complex FFT_output[FFT_Size], FFT_output_PLACP[FFT_Size];

    // Variable for intermediate outputs
    float complex FFT_rev[FFT_Size];

    XTime time_PS_start, time_PS_end;
    XTime time_PLACP_start, time_PLACP_end;   // PL time calculations

    float time_processor = 0;
    float time_FPGAACP = 0;
    float curr_time = 0;

    // ACP DMA Initialization
    XAxiDma_Config *DMA_confptracp;   // DMA configuration pointer
    XAxiDma AxiDMAacp;                // DMA instance pointer

    // Copy the DMA information (received from hardware in xparameters.h file)
    DMA_confptracp = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
    status = XAxiDma_CfgInitialize(&AxiDMAacp, DMA_confptracp);
    if (status != XST_SUCCESS)
    {
        printf("ACP DMA Init Failed\t\n");
        return XST_FAILURE;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set up Interrupt system
    status = SetupIntrSystem(&INTCInst, &AxiDMAacp, XPAR_FABRIC_AXIDMA_0_MM2S_INTROUT_VEC_ID, XPAR_FABRIC_AXIDMA_0_S2MM_INTROUT_VEC_ID);
    if (status != XST_SUCCESS)
	{
		printf("Failed intr setup\r\n");
		return XST_FAILURE;
	}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize flags before start transfer test
    MM2SDone = 0;
    S2MMDone = 0;
    Error = 0;


    for (int k = 0; k < EXPT; k++)
    {
        XTime seed_value;
        XTime_GetTime(&seed_value);
        srand(seed_value);

        // Generate random numbers
        for (int i = 0; i < FFT_Size; i++)
        {
            FFT_input[i] = (rand() % 2000) + (rand() % 2000) * I;
        }

        // FFT calculation on PS
        XTime_SetTime(0);
        XTime_GetTime(&time_PS_start);   // Capture the timer value at the start

        // As discussed in the handout, FFT involves two tasks:
        // 1) Reorder of the inputs to get output in the normal order
        // 2) Multiplications using multi-stage butterfly approach
        InputReorder(FFT_input, FFT_rev);   // Task 1
        FFTStages(FFT_rev, FFT_output);     // Task 2

        XTime_GetTime(&time_PS_end);   // Capture the timer value at the end
        curr_time = ((float)1.0 * (time_PS_end - time_PS_start) / (COUNTS_PER_SECOND / 1000000));
        time_processor = time_processor + curr_time;

        //printf("Execution Time for PS in Micro-Seconds for %d iteration: %f\n", k, curr_time);

        XTime_SetTime(0);
        XTime_GetTime(&time_PLACP_start);   // Capture the timer value at the start

//        Xil_DCacheFlushRange((UINTPTR)FFT_input, (sizeof(float complex) * FFT_Size));

        status = XAxiDma_SimpleTransfer(&AxiDMAacp, (UINTPTR)FFT_output_PLACP,
                                        (sizeof(float complex) * FFT_Size), XAXIDMA_DEVICE_TO_DMA);
        status = XAxiDma_SimpleTransfer(&AxiDMAacp, (UINTPTR)FFT_input,
                                        (sizeof(float complex) * FFT_Size), XAXIDMA_DMA_TO_DEVICE);

//        Xil_DCacheInvalidateRange((UINTPTR)FFT_output_PLACP, (sizeof(float complex) * FFT_Size));

        //////////////////////////////////////////////////////////////////////////////////////////
		// Wait for completion of DMA transfer
		while (!MM2SDone && !S2MMDone && !Error)
		{}
		if (Error)
		{
			if (!MM2SDone)
				printf("MM2S is Failed\t\n");
			if (!S2MMDone)
				printf("S2MM is Failed\t\n");
			break;
		}
		//////////////////////////////////////////////////////////////////////////////////////////



        XTime_GetTime(&time_PLACP_end);   // Capture the timer value at the end

        curr_time = ((float)1.0 * (time_PLACP_end - time_PLACP_start) / (COUNTS_PER_SECOND / 1000000));
        time_FPGAACP = time_FPGAACP + curr_time;

        //printf("Execution Time for PL ACP INTR in Micro-Seconds for %d iteration: %f\n", k, curr_time);
    }

    printf("Execution Time for PS in Micro-Seconds : %f\n", time_processor / EXPT);
    printf("Average Execution Time for PL ACP INTR in Micro-Seconds : %f\n", time_FPGAACP / EXPT);

    //////////////////////////////////////////////////////////////////////////////////////////
//    DisconnIntrSystem(&INTCInst, XPAR_FABRIC_AXIDMA_0_MM2S_INTROUT_VEC_ID, XPAR_FABRIC_AXIDMA_0_S2MM_INTROUT_VEC_ID);
    //////////////////////////////////////////////////////////////////////////////////////////

    return 0;
}

int FFTPSvsACP_poll()
{
    int status;

    // For FFT_Size point FFT, define the input.
    // You may modify the code to take the input from user via UART
    float complex FFT_input[FFT_Size];   // = {11+23*I,32+10*I,91+94*I,15+69*I,47+96*I,44+12*I,96+17*I,49+58*I};

    // FFT output will be stored in this variable
    float complex FFT_output[FFT_Size], FFT_output_PLACP[FFT_Size];

    // Variable for intermediate outputs
    float complex FFT_rev[FFT_Size];

    XTime time_PS_start, time_PS_end;
    XTime time_PLACP_start, time_PLACP_end;   // PL time calculations

    float time_processor = 0;
    float time_FPGAACP = 0;
    float curr_time = 0;

    // ACP DMA Initialization
    XAxiDma_Config *DMA_confptracp;   // DMA configuration pointer
    XAxiDma AxiDMAacp;                // DMA instance pointer

    // Copy the DMA information (received from hardware in xparameters.h file)
    DMA_confptracp = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
    status = XAxiDma_CfgInitialize(&AxiDMAacp, DMA_confptracp);
    if (status != XST_SUCCESS)
    {
        printf("ACP DMA Init Failed\t\n");
        return XST_FAILURE;
    }

    for (int k = 0; k < EXPT; k++)
    {
        XTime seed_value;
        XTime_GetTime(&seed_value);
        srand(seed_value);

        // Generate random numbers
        for (int i = 0; i < FFT_Size; i++)
        {
            FFT_input[i] = (rand() % 2000) + (rand() % 2000) * I;
        }

        // FFT calculation on PS
        XTime_SetTime(0);
        XTime_GetTime(&time_PS_start);   // Capture the timer value at the start

        // As discussed in the handout, FFT involves two tasks:
        // 1) Reorder of the inputs to get output in the normal order
        // 2) Multiplications using multi-stage butterfly approach
        InputReorder(FFT_input, FFT_rev);   // Task 1
        FFTStages(FFT_rev, FFT_output);     // Task 2

        XTime_GetTime(&time_PS_end);   // Capture the timer value at the end
        curr_time = ((float)1.0 * (time_PS_end - time_PS_start) / (COUNTS_PER_SECOND / 1000000));
        time_processor = time_processor + curr_time;

        //printf("Execution Time for PS in Micro-Seconds for %d iteration: %f\n", k, curr_time);

        XTime_SetTime(0);
        XTime_GetTime(&time_PLACP_start);   // Capture the timer value at the start

//        Xil_DCacheFlushRange((UINTPTR)FFT_input, (sizeof(float complex) * FFT_Size));

        status = XAxiDma_SimpleTransfer(&AxiDMAacp, (UINTPTR)FFT_output_PLACP,
                                        (sizeof(float complex) * FFT_Size), XAXIDMA_DEVICE_TO_DMA);
        status = XAxiDma_SimpleTransfer(&AxiDMAacp, (UINTPTR)FFT_input,
                                        (sizeof(float complex) * FFT_Size), XAXIDMA_DMA_TO_DEVICE);

//        Xil_DCacheInvalidateRange((UINTPTR)FFT_output_PLACP, (sizeof(float complex) * FFT_Size));

        status = XAxiDma_ReadReg(XPAR_AXI_DMA_0_BASEADDR, 0x04) & 0x00000002;
		while (status != 0x00000002)
		{
			status = XAxiDma_ReadReg(XPAR_AXI_DMA_0_BASEADDR, 0x04) & 0x00000002;
		}

		status = XAxiDma_ReadReg(XPAR_AXI_DMA_0_BASEADDR, 0x34) & 0x00000002;
		while (status != 0x00000002)
		{
			status = XAxiDma_ReadReg(XPAR_AXI_DMA_0_BASEADDR, 0x34) & 0x00000002;
		}

        XTime_GetTime(&time_PLACP_end);   // Capture the timer value at the end

        curr_time = ((float)1.0 * (time_PLACP_end - time_PLACP_start) / (COUNTS_PER_SECOND / 1000000));
        time_FPGAACP = time_FPGAACP + curr_time;

        //printf("Execution Time for PL ACP POOL in Micro-Seconds for %d iteration: %f\n", k, curr_time);
    }

    printf("Execution Time for PS in Micro-Seconds : %f\n", time_processor / EXPT);
    printf("Average Execution Time for PL ACP POOL in Micro-Seconds : %f\n", time_FPGAACP / EXPT);



    return 0;
}

int main()
{
    init_platform();

    FFTPSvsACP_poll();   // Write a function to compare the performance of FFT on PS and ACP based PL
    FFTPSvsACP_intr();

    cleanup_platform();
    return 0;
}

