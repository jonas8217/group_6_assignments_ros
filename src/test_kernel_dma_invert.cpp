#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
// #include <stdlib.h>
#include <iostream>

#include <time.h>
#include <chrono>

#include <math.h>

#include "lib/AXI-DMA-UIO-cpp-driver/include/axi_dma_controller.h"
#include "lib/ReservedMemory-LKM-and-UserSpaceAPI/reserved_mem.hpp"
#include "lib/Invert_v1_0/src/xinvert.c"
#include "lib/Invert_v1_0/src/xinvert_sinit.c"
#include "lib/Invert_v1_0/src/xinvert_linux.c"

#define DEVICE_FILENAME "/dev/reservedmemLKM"
#define LENGTH 240 //(800*600*4)
#define LENGTH_INPUT 	(LENGTH*3/4)
#define LENGTH_OUTPUT	(LENGTH/4)
// #define LENGTH 0x007fffff // Length in bytes
#define P_START 0x70000000
#define TX_OFFSET 0
#define RX_OFFSET LENGTH_INPUT // Should be (600×800×3)×(3÷4)÷4=270000 because it needs to be a whole number

//#define i_P_START 0
//#define i_LENGTH 1
//#define i_inp_buffER_PTR_L 2
//#define i_inp_buffER_PTR_H 3

#define UIO_DMA_N 1

#define XST_FAILURE		1L	//This is nice to have :)

#define inputVal		0xebebebeb
#define outputVal		0x14141414

// clock_t t;
std::chrono::_V2::system_clock::time_point t1;
void start_timer()
{
	t1 = std::chrono::high_resolution_clock::now();
	// std::cout << "Start timer" << std::endl;
}
double stop_timer()
{
	auto t2 = std::chrono::high_resolution_clock::now();
	auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
	std::chrono::duration<double, std::milli> ms_double = t2 - t1;
	// std::cout << "Duration: " << ms_double.count() << "ms [" << (float)LENGTH / 1000000. << "MB]" << std::endl;
	return ms_double.count();
}

void print_mem(void *virtual_address, int byte_count)
{
	char *data_ptr = (char *)virtual_address;

	for (int i = 0; i < byte_count; i++)
	{
		printf("%02X", data_ptr[i]);

		// print a space every 4 bytes (0 indexed)
		if (i % 4 == 3)
		{
			printf(" ");
		}
	}

	printf("\n");
}

int main()
{
	int ret = 0;
	double total_t = 0;
	double tmp = 0;
	printf("\nHello World! - Running DMA transfer test application with specified memory.\n\n");

	Reserved_Mem pmem;
	AXIDMAController dma(UIO_DMA_N, 0x10000);
	
	printf("\nInitalizing invert ip\n");
	int Status;
	XInvert invertIP;
	Status = XInvert_Initialize(&invertIP, "Invert");
	
	if (Status != XST_SUCCESS) {
		printf("Initialization failed %d\r\n", Status);
		return XST_FAILURE;
	}
	printf("\r\n--- Invert Intialized --- \r\n");


	uint32_t *inp_buff = (uint32_t *)malloc(LENGTH_INPUT);
	if (inp_buff == NULL)
	{
		printf("could not allocate user buffer\n");
		return -1;
	}
	
	for (int i = 0; i < (LENGTH_INPUT) / sizeof(uint32_t); i++)
		inp_buff[i] = inputVal;

	printf("User memory reserved and filled\n");
	
	tmp = 0;
	start_timer();
	// ret = write(reserved_mem_fd, write_info_LKM, sizeof(write_info_LKM));
	pmem.transfer(inp_buff, TX_OFFSET, LENGTH_INPUT);
	total_t += stop_timer();
	std::cout << "Data transfered to reserved memory: " << total_t << "ms [" << (float)LENGTH_INPUT / 1000000. << "MB]" << std::endl;

	start_timer();
	printf("Reset the DMA.\n");
	dma.MM2SReset();
	dma.S2MMReset();

	printf("Check MM2S status.\n");
	DMAStatus mm2s_status = dma.MM2SGetStatus();
	printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
	printf("Check S2MM status.\n");
	DMAStatus s2mm_status = dma.S2MMGetStatus();
	printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	printf("\n");

	printf("Halt the DMA.\n");
	dma.MM2SHalt();
	dma.S2MMHalt();

	printf("Check MM2S status.\n");
	mm2s_status = dma.MM2SGetStatus();
	printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
	printf("Check S2MM status.\n");
	s2mm_status = dma.S2MMGetStatus();
	printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	printf("\n");

	printf("Enable all interrupts.\n");
	dma.MM2SInterruptEnable();
	dma.S2MMInterruptEnable();

	printf("Check MM2S status.\n");
	mm2s_status = dma.MM2SGetStatus();
	printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
	printf("Check S2MM status.\n");
	s2mm_status = dma.S2MMGetStatus();
	printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	printf("\n");

	printf("Writing source address of the data from MM2S in DDR...\n");
	dma.MM2SSetSourceAddress(P_START + TX_OFFSET);
	printf("Check MM2S status.\n");
	mm2s_status = dma.MM2SGetStatus();
	printf("MM2S status: %s\n", mm2s_status.to_string().c_str());

	printf("Writing the destination address for the data from S2MM in DDR...\n");
	dma.S2MMSetDestinationAddress(P_START + RX_OFFSET);
	printf("Check S2MM status.\n");
	s2mm_status = dma.S2MMGetStatus();
	printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	printf("\n");

	while(!XInvert_IsReady(&invertIP)) {
		// wait
	}
	printf("\nIp ready\n");

	// Start IP
	XInvert_Start(&invertIP);

	dma.MM2SStart();
	printf("Run the MM2S channel.\n");
	printf("Check MM2S status.\n");
	mm2s_status = dma.MM2SGetStatus();
	printf("MM2S status: %s\n", mm2s_status.to_string().c_str());

	printf("Run the S2MM channel.\n");
	dma.S2MMStart();
	printf("Check S2MM status.\n");
	s2mm_status = dma.S2MMGetStatus();
	printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	printf("\n");

	printf("\nWriting MM2S transfer length of %i bytes...\n", LENGTH_INPUT);
	dma.MM2SSetLength(LENGTH_INPUT); //! WIll only work up to 2^23
	printf("Check MM2S status.\n");
	mm2s_status = dma.MM2SGetStatus();
	printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
	printf("Writing S2MM transfer length of %i bytes...\n",LENGTH_OUTPUT);
	dma.S2MMSetLength(LENGTH_OUTPUT);
	printf("Check S2MM status.\n");
	s2mm_status = dma.S2MMGetStatus();
	printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	printf("\n");
	
	tmp = stop_timer();
	total_t+=tmp;
	std::cout << "\nDMA setup done, transfer begun: " << tmp << "ms [" << (float)LENGTH_INPUT / 1000000. << "MB]\n" << std::endl;

	start_timer();
	printf("...Waiting for MM2S synchronization...\n");

	// bool first = true;
	while (!dma.MM2SIsSynced())
	{
		// if (first)
		// {
		// 	printf("Not synced yet...\n");
		// 	first = false;
		// }
	}

	tmp = stop_timer();
	total_t += tmp;
	std::cout << "\nData transfered to transfered by DMA: " << tmp << "ms [" << (float)LENGTH_INPUT / 1000000. << "MB]\n" << std::endl;

	printf("Check MM2S status.\n");
	mm2s_status = dma.MM2SGetStatus();
	printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
	printf("Waiting for S2MM sychronization...\n");
	while(!dma.S2MMIsSynced()) {
		//printf("Not synced yet...\n");
	}

	printf("Check S2MM status.\n");
	s2mm_status = dma.S2MMGetStatus();
	printf("S2MM status: %s\n", s2mm_status.to_string().c_str());

	// Wait for ip to finish
	printf("Wait for ip to finish");
	while(!XInvert_IsDone(&invertIP)) {
		// Wait
	}
	printf("\nIp done!\n");

	// write_info_LKM[i_P_START] = 0; //! CHECK update read
	// write_info_LKM[i_LENGTH] = 100;
	// write_info_LKM[i_K_START] = 10;
	// ret = read(reserved_mem_fd, write_info_LKM, sizeof(write_info_LKM));
	// if (ret < 0)
	// {
	//     printf("read error!\n");
	//     ret = errno;
	//     goto out;
	// }

	//uint32_t *out_buff = (uint32_t *)malloc(LENGTH_OUTPUT);
	//pmem.gather(out_buff, RX_OFFSET, LENGTH_OUTPUT);
	//print_mem(out_buff, LENGTH_OUTPUT);
	printf("\n\n");
	
	//pmem.gather(inp_buff, TX_OFFSET, LENGTH_INPUT);
	uint32_t *io_buff = (uint32_t *)malloc(LENGTH*2);
	
	pmem.gather(io_buff, TX_OFFSET, LENGTH*2);
	
	for (int i = 0; i < (LENGTH*2) / sizeof(uint32_t); i++) {
		printf("\nio_buff: %i %x\n\r", i, io_buff[i]);
	}
	
	//for (int i = 0; i < LENGTH_INPUT / sizeof(uint32_t); i++) {
		//if (inp_buff[i] != inputVal) {
	//		printf("\nFailure in inp_buff: %i %x\n\r", i, inp_buff[i]);
			//break;
		//}
	//}

	//for (int i = 0; i < LENGTH_OUTPUT / sizeof(uint32_t); i++) {
		//if (out_buff[i] != outputVal) {
	//		printf("\nFailure in out_buff: %i %x\n\r", i, out_buff[i]);
			//break;
		//}
	//}
	

	// printf("Data in buffer after read\n");
	// for (int i = 0; i < 20; i++)
	// {
	//     printf("%i ", p[i]);
	// }
	printf("\nALL DONE!\n");

	std::cout << "\nTotal duration of transfer: " << total_t << "ms [" << (float)LENGTH_INPUT / 1000000. << "MB]" << std::endl;
	return ret;
}
