#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgcodecs.hpp>


#include "lib/AXI-DMA-UIO-cpp-driver/include/axi_dma_controller.h"
#include "lib/ReservedMemory-LKM-and-UserSpaceAPI/reserved_mem.hpp"
#include "lib/Invert_v1_0/src/xinvert.c"
#include "lib/Invert_v1_0/src/xinvert_sinit.c"
#include "lib/Invert_v1_0/src/xinvert_linux.c"

#define UIO_INVERT_Nsjjk 0L

#define UIO_DMA_N 1

#define XST_FAILURE		1L	//This is nice to have :)

#define DEVICE_FILENAME "/dev/reservedmemLKM"
#define IMAGE_WIDTH		800
#define IMAGE_HEIGHT	600
#define LENGTH IMAGE_WIDTH*IMAGE_HEIGHT*4 //(800*600*4) // Number of bytes (rgb + grayscale)
#define LENGTH_INPUT 	LENGTH*3/4 // Number of bytes for input (3/4 because rgb)
#define LENGTH_OUTPUT	LENGTH/4 // Number of bytes for output (1/4 because grayscale)
// #define LENGTH 0x007fffff // Length in bytes
#define P_START 0x70000000
#define TX_OFFSET 0
#define RX_OFFSET_BYTES LENGTH_INPUT
#define RX_OFFSET_32 RX_OFFSET_BYTES/4 // This needs to be a whole number, otherwise input in ram is overwritten!

class ImageSubscriber : public rclcpp::Node
{
	public:
		ImageSubscriber() : Node("image_subscriber"), dma(AXIDMAController(UIO_DMA_N, 0x10000)) {
			RCLCPP_INFO(this->get_logger(), "Initializing ImageSubscriber node");

			RCLCPP_INFO(this->get_logger(), "Starting camera subscription");

			camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
					"/image_raw",
					10,
					std::bind(&ImageSubscriber::onImageMsg, this, std::placeholders::_1)
			);

			image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
				"/image_processed",
				10
			);

            init_IPs_and_setup();

		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
		

		cv::Mat inp_img;
        cv::Mat out_img;


        Reserved_Mem pmem;
        AXIDMAController dma;
        XInvert invertIP;

        uint8_t *inp_buff;
        uint8_t *out_buff;

        int init_IPs_and_setup(){

            printf("\nInitalizing IPs\n");
            int Status;
            Status = XInvert_Initialize(&invertIP, "Invert");
            
            if (Status != XST_SUCCESS) {
                printf("Invert initialization failed %d\r\n", Status);
                return XST_FAILURE;
            }
            printf("\r\n--- IPs Intialized --- \r\n");

            inp_buff = (uint8_t *)malloc(LENGTH_INPUT);
            if (inp_buff == NULL)
            {
                printf("could not allocate user buffer\n");
                return -1;
            }
            out_buff = (uint8_t *)malloc(LENGTH_OUTPUT);
            if (out_buff == NULL)
            {
                printf("could not allocate user buffer\n");
                return -1;
            }

            dma.MM2SReset();
            dma.S2MMReset();

            dma.MM2SInterruptEnable();
    		dma.S2MMInterruptEnable();

            dma.MM2SSetSourceAddress(P_START + TX_OFFSET);
            dma.S2MMSetDestinationAddress(P_START + RX_OFFSET_BYTES);

            dma.MM2SSetLength(LENGTH_INPUT);
            dma.S2MMSetLength(LENGTH_OUTPUT);

        }


		void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "Received image!");

			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
			inp_img = cv_ptr->image;
            //cv::OutputArray a();
            //inp_img.convertTo(a,CV_8UC3);

            printf("%s",cv_ptr->encoding.c_str());
            
            return;
            inp_buff = (uint8_t *)inp_img.data;
			// Send data to ram


			RCLCPP_INFO(this->get_logger(), "Successfully loaded image");

			sensor_msgs::msg::Image::SharedPtr processed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), msg->encoding, inp_img).toImageMsg();
			
			image_publisher_->publish(*processed_image_msg.get());
		}

        
        void loadImage(uint8_t *inp_buff) {
            uint32_t *Image = (uint32_t *)inp_buff;

            // Image count
            volatile int Image_Count = 0;

            // Get test data
            // while(std::getline(Test_File, Test_Data)){
            //     Image[Image_Count] = (uint32_t)std::stoul(Test_Data);
            //     Image_Count++;
            // }
        }

        void run_Invert_IP(){

            pmem.transfer(inp_buff, TX_OFFSET, LENGTH_INPUT);

            dma.MM2SHalt();
            dma.S2MMHalt();

            while(!XInvert_IsReady(&invertIP)) {
                // wait
            }
            // Start IP
            XInvert_Start(&invertIP);
            dma.MM2SStart();
            dma.S2MMStart();

            while (!dma.MM2SIsSynced()) {}

            while (!dma.S2MMIsSynced()) {}
            
            while(!XInvert_IsDone(&invertIP)) {}

            pmem.gather(out_buff, RX_OFFSET_32, LENGTH_OUTPUT);

        }



};

int main(int argc, char *argv[])
{
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<ImageSubscriber>());

	rclcpp::shutdown();
	return 0;
}
