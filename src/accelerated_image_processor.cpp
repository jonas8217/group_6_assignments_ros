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

            out_img.create(600,800,CV_8U);

            init_IPs_and_setup();

		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
		

		cv::Mat inp_img;
        cv::Mat inp_img_rgb;
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


        }


		void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "Received image!");

			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
			inp_img = cv_ptr->image;
            //cv::OutputArray a();
            //inp_img.convertTo(a,CV_8UC3);

            loadImage();

			RCLCPP_INFO(this->get_logger(), "Successfully loaded image");

            RCLCPP_INFO(this->get_logger(), "Running IP");

            run_Invert_IP();

            RCLCPP_INFO(this->get_logger(), "IP completed");

            outputImage();

			sensor_msgs::msg::Image::SharedPtr processed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", out_img).toImageMsg();
			
			image_publisher_->publish(*processed_image_msg.get());
		}

        
        void loadImage() {
            int i = 0;
            cv::cvtColor(inp_img,inp_img_rgb,cv::COLOR_YUV2RGB_UYVY);
            for (int y = 0; y < inp_img_rgb.rows; y++){
                for (int x = 0; x < inp_img_rgb.cols; x++){
                    for (int c = 0; c < 3; c++){
                        inp_buff[y*inp_img_rgb.cols*3+x*3+c + (3-(i%4)*2)] = inp_img_rgb.at<cv::Vec3b>(x,y)[c];
                        i++;
                    }
                }
            }
        }

        void outputImage() {
            int i = 0;
            for (int y = 0; y < out_img.rows; y++){
                for (int x = 0; x < out_img.cols; x++){
                    out_img.at<uint8_t>(x,y) = out_buff[y*out_img.cols+x + (3-(i%4)*2)];
                    i++;
                }
            }
        }

        void run_Invert_IP(){

            printf("test1");
            pmem.transfer(inp_buff, TX_OFFSET, LENGTH_INPUT);
            printf("test2");
            dma.MM2SHalt();
            dma.S2MMHalt();
            printf("test3");
            while(!XInvert_IsReady(&invertIP)) {}
            printf("test4");
            XInvert_Start(&invertIP);
            printf("test5");
            dma.MM2SStart();
            dma.S2MMStart();

            dma.MM2SSetLength(LENGTH_INPUT);
            dma.S2MMSetLength(LENGTH_OUTPUT);
            printf("test6");
            while (!dma.MM2SIsSynced()) {}
            printf("test7");
            while (!dma.S2MMIsSynced()) {}
            printf("test8");
            while(!XInvert_IsDone(&invertIP)) {}
            printf("test9");
            pmem.gather(out_buff, RX_OFFSET_32, LENGTH_OUTPUT);
            printf("test10");
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
