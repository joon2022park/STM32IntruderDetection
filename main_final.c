/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <time.h>
#include <string.h>
#include <stdio.h>
#include "ov7670.h"
#include <stdlib.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMAGE_WIDTH 174
#define IMAGE_HEIGHT 144
#define MAX_LABELS 255

#define MOTION_DETECTED_THRESHOLD 1100000
#define MOTION_CLEARED_THRESHOLD 900000
#define MOTION_DETECTED_TIME 3000

static uint32_t last_motion_time = 0;
static uint8_t motion_detected = 0;
static uint8_t pinValid = 0;

#define FRAME_SIZE (IMG_ROWS * IMG_COLS)
#define HALF_FRAME_SIZE (FRAME_SIZE / 2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
//FSM VARIABLES
#define PREAMBLE "\r\n!START!\r\n"
#define DELTA_PREAMBLE "\r\n!DELTA!\r\n"
#define SUFFIX "!END!\r\n"

uint16_t snapshot_buff[IMG_ROWS * IMG_COLS];
uint8_t old_snapshot_buff[IMG_ROWS * IMG_COLS];

uint8_t tx_buff[sizeof(PREAMBLE) + 2 * IMG_ROWS * IMG_COLS + sizeof(SUFFIX)];
size_t tx_buff_len = 0;

volatile uint8_t dma_flag = 0;
uint8_t streaming = 0;
uint8_t old_diff = 0;

//KEYPAD FSM VARIABLES
typedef enum {IDLE = 0, S1, S2, S3, VALIDPIN, ACCEPTED} states;
char digit;
char prevKey = '\0', returnKey = '\0';
//char PIN[] = "7428";
char PIN[5];
char buffer[50];
int8_t current_row = -1, current_col = -1, wasPressed = 0;

//CAMERA VARIABLES
//volatile uint8_t dma_flag = 0;
uint16_t snapshot_buff[IMG_ROWS * IMG_COLS];
uint8_t gradientX[IMG_ROWS][IMG_COLS];
uint8_t gradientY[IMG_ROWS][IMG_COLS];		//reuse this for  max and min arrays in detect human
uint8_t res_buff[IMG_ROWS][IMG_COLS];

#define WARM_UP_TIME 5000
static uint32_t start_time = 0;

// uint8_t binaryImage[IMAGE_HEIGHT][IMAGE_WIDTH]; 		use res_buff instead
//uint8_t labelImage[IMAGE_HEIGHT][IMAGE_WIDTH];		reuse gradient X
//uint16_t equivalenceTable[MAX_LABELS]; 		resue snapshot buff
uint8_t humanDetected = 0;
uint16_t* equivalenceTable;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef print_msg_DMA(char * msg) {
  return HAL_UART_Transmit_DMA(&huart3, (uint8_t *)msg, strlen(msg));
}

HAL_StatusTypeDef uart_send_bin(uint8_t * buff, unsigned int len) {
  return HAL_UART_Transmit_DMA(&huart3, (uint8_t *)buff, len);
}

void printout(char message[]){
  print_msg(message);
}

void print_msg(char * msg) {
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}

void send_random_code(void) { //send random authorization code over Bluetooth
    char message[10];  // Buffer to store the random number
    int random_code = (rand() % 9000) + 1000;  // Generates a number between 1000-9999
    sprintf(PIN, "%04d", random_code);				// Store just the 4-digit code as a string in PIN
    sprintf(message, "Code: %d\r\n", random_code);  // Convert to string
    HAL_UART_Transmit(&huart3, (uint8_t*)message, strlen(message), 1000);  // Send via Bluetooth
}

void initEquivalenceTable(uint16_t* table) {
    for (int i = 0; i < MAX_LABELS; i++) {
    	table[i] = i;
    }
}

// Find the root label using path compression
uint16_t findRoot(uint16_t label, uint16_t* equivalenceTable) {
	if (label >= MAX_LABELS) {
	        char msg[100];
	        sprintf(msg, "Error: findRoot received invalid label %d\r\n", label);
	        print_msg(msg);
	        return 0; // Return 0 as a safe label
	}
	while (label != equivalenceTable[label]) {
        // Path compression to flatten the tree
        equivalenceTable[label] = equivalenceTable[equivalenceTable[label]];
        label = equivalenceTable[label];
    }
    return label;
}

// Union of two labels
void unionLabels(uint16_t label1, uint16_t label2, uint16_t* equivalenceTable) {
	if (label1 >= MAX_LABELS || label2 >= MAX_LABELS) {
		        char msg[100];
		        sprintf(msg, "Error: union labels received invalid label \r\n");
		        print_msg(msg);
		        return; // Return 0 as a safe label
		}
    uint16_t root1 = findRoot(label1, equivalenceTable);
    uint16_t root2 = findRoot(label2, equivalenceTable);

    if (root1 != root2) {
        equivalenceTable[root2] = root1; // Union the components
    }
}

//first pass of connected component labelling
void firstPass() {
	memset(gradientX, 0, sizeof(gradientX));
	int isCleared = 1; // Assume cleared
	for (int i = 0; i < IMAGE_HEIGHT; i++) {
	    for (int j = 0; j < IMAGE_WIDTH; j++) {
	        if (gradientX[i][j] != 0) {
	            char msg[50];
	            sprintf(msg, "Error: gradientX[%d][%d] is not zero, value: %d\r\n", i, j, gradientX[i][j]);
	            print_msg(msg);
	            isCleared = 0;
	        }
	    }
	}

	//print_msg("gradient X reset\r\n");
    uint16_t nextLabel = 1;
    for (int i = 0; i < IMAGE_HEIGHT; i++) {
        for (int j = 0; j < IMAGE_WIDTH; j++) {
        	if (nextLabel >= MAX_LABELS) {
        	    print_msg("Error: Exceeded MAX_LABELS\r\n");
        	    return;
        	}
        	if (i < 0 || i >= IMAGE_HEIGHT || j < 0 || j >= IMAGE_WIDTH) {
        	    print_msg("Error: Out of bounds access!\r\n");
        	    return;
        	}

        	if (res_buff[i][j] != 0 && res_buff[i][j] != 255) {
        		char msg[100];
				sprintf(msg, "Unexpected value in res_buff[%d][%d] = %d\r\n", i, j, res_buff[i][j]);
				print_msg(msg);
				//print_msg("Unexpected value in res_buff[%d][%d] = %d\r\n", i, j, res_buff[i][j]);
        	}
        	//else { print_msg("all good \r\n");}
            if (res_buff[i][j] == 255) {
            	 print_msg("Test2");

                uint16_t leftLabel = (j > 0) ? gradientX[i][j-1] : 0;
                uint16_t topLabel = (i > 0) ? gradientX[i-1][j] : 0;
                uint16_t topLeftLabel = (i > 0 && j > 0) ? gradientX[i-1][j-1] : 0;
                uint16_t topRightLabel = (i > 0 && j < IMAGE_WIDTH - 1) ? gradientX[i-1][j+1] : 0;

                // Collect non-zero neighbors
                uint16_t neighbors[4] = {leftLabel, topLabel, topLeftLabel, topRightLabel};
                uint16_t minLabel = MAX_LABELS;
                int hasLabel = 0;

                for (int k = 0; k < 4; k++) {
                    if (neighbors[k] > 0 && neighbors[k] < minLabel) {
                        minLabel = neighbors[k];
                        hasLabel = 1;
                        print_msg("Test2");
                    }
                }

                if (!hasLabel) {
                    if (nextLabel >= MAX_LABELS) {
                        print_msg("Error: Exceeded MAX_LABELS while assigning label\r\n");
                        return;
                    }
                    gradientX[i][j] = nextLabel++;
                } else {
                	gradientX[i][j] = minLabel;

                    for (int k = 0; k < 4; k++) {
                        if (neighbors[k] > 0 && neighbors[k] != minLabel) {
                            unionLabels(minLabel, neighbors[k], equivalenceTable);
                        }
                    }
                }
            }
        }
    }
    print_msg("exiting first pass\r\n");
}

//Analyze created components to detect human silhouette
void detectHuman() {
		humanDetected = 0;
		uint8_t label;
		char msg[100];

	    uint8_t* minX = (uint8_t*)gradientY;
	    uint8_t* minY = minX + MAX_LABELS;
	    uint8_t* maxX = minY + MAX_LABELS;
	    uint8_t* maxY = maxX + MAX_LABELS;
	    uint8_t* area = (uint8_t*)(maxY + MAX_LABELS);

	    for (int i = 0; i < MAX_LABELS; i++) {
	        minX[i] = IMAGE_WIDTH;
	        minY[i] = IMAGE_HEIGHT;
	        maxX[i] = 0;
	        maxY[i] = 0;
	        area[i] = 0;
	    }

    // Calculate bounding boxes and areas
    for (int i = 0; i < IMAGE_HEIGHT; i++) {
        for (int j = 0; j < IMAGE_WIDTH; j++) {
             label = gradientX[i][j];

             if (label > 0) {
            	 //print_msg("if condition entered\r\n");
                 area[label]++;
                 if (j < minX[label]) minX[label] = j;
                 if (j > maxX[label]) maxX[label] = j;
                 if (i < minY[label]) minY[label] = i;
                 if (i > maxY[label]) maxY[label] = i;
             }

        }
    }

    int printedCount = 0;
    uint8_t count = 0;
    // Evaluate for human-like components
    for (int label = 1; label < MAX_LABELS; label++) {

		printedCount++;
		if (printedCount >= 15) {
			break; // Stop after printing 5 labels
		}
        if (area[label] > 0) {
        	count++;
            int width = maxX[label] - minX[label] + 1;
            int height = maxY[label] - minY[label] + 1;
            float aspectRatio = (float)height / (float)width;
            sprintf(msg, "Label %d -> MinX: %d, MinY: %d, MaxX: %d, MaxY: %d, Area: %d, Aspect ratio: %d\r\n",
                	                            label, minX[label], minY[label], maxX[label], maxY[label], area[label], (int)aspectRatio);
            print_msg(msg);

            // Print values for the first 5 labels with area > 0


            if (aspectRatio >= 3.0 && aspectRatio <= 3.6) {
//            	char msg[50]; // Allocate a buffer for the message
//            	sprintf(msg, "Human detected with label %d\r\n", label);
            	print_msg("HUMAN DETECTED\r\n");
            	humanDetected = 1;
            }
    }
    //sprintf(msg, "count with area > 0 %d\r\n", count);


    //char msg[100]; // Buffer to store the formatted message

    }
    if (!humanDetected) {
    	print_msg("HUMAN NOT DETECTED\r\n");
    }
}

void print_buf() {
	//print_msg("inside print buf \r\n");
	HAL_DCMI_Suspend(&hdcmi);

  // Create a new buffer from the snapshot_buffer than the DCMI copied the 16-bit pixel values into.
  uint8_t *buffer = (uint8_t *) snapshot_buff;

  // Reset tx_buff_len
  tx_buff_len = 0;

  // Add the START preamble message to the start of the buffer for the serial-monitor program.
  for (int i = 0; i < strlen(PREAMBLE); i++) {
	  tx_buff[tx_buff_len++] = PREAMBLE[i];
  }


  // Write code to copy every other byte from the main frame buffer to
  // our temporary buffer (this converts the image to grey scale)

  // Convert image to grayscale and store in tx_buff
  for (int i = 0; i < IMG_ROWS * IMG_COLS; i++) {
          tx_buff[tx_buff_len++] = (uint8_t)((snapshot_buff[i] >> 8) & 0xFF); // Extract Y (brightness)
  }

  //*********with data truncation******
  // Pack two 4-bit pixels into each byte
  /*for (int i = 0; i < IMG_ROWS * IMG_COLS; i += 2) {
	  uint8_t pixel1 = (snapshot_buff[i] >> 12) & 0x0F;   // Extract top 4 bits of first pixel
	  uint8_t pixel2 = (snapshot_buff[i + 1] >> 12) & 0x0F; // Extract top 4 bits of second pixel
	  tx_buff[tx_buff_len++] = (pixel1 << 4) | pixel2;  // Pack into one byte
  }*/

  //*********with data truncation + RLE******
  // Run-Length Encoding (RLE) for 4-bit pixels
  uint8_t prev_pixel = (snapshot_buff[0] >> 12) & 0x0F;  // Get first pixel (top 4 bits)
  uint8_t count = 1;

  for (int i = 1; i <= IMG_ROWS * IMG_COLS; i++) {
	  uint8_t curr_pixel = (snapshot_buff[i] >> 12) & 0x0F;  // Extract top 4 bits

	  if (curr_pixel == prev_pixel && count < 15) {
		  count++;  // Increment run-length count
	  } else {
		  // Store (pixel, count) pair
		  //tx_buff[tx_buff_len++] = (prev_pixel << 4) | count;

		  // Store (pixel, count) pair
		  if (tx_buff_len < sizeof(tx_buff)) {
			  tx_buff[tx_buff_len++] = (prev_pixel << 4) | count;
		  }

		  prev_pixel = curr_pixel;
		  count = 1;
	  }
  }

  // Load the END suffix message to the end of the message.
  for (int i = 0; i < strlen(SUFFIX); i++) {
    tx_buff[tx_buff_len++] = SUFFIX[i];
  }
  //print_msg("tranmitting \r\n");

  //HAL_Delay(50);
  // Once the data is copied into the buffer, call the function to send it via UART.
  if (HAL_UART_GetState(&huart3) == HAL_UART_STATE_READY) {
	  //print_msg("if condition is true \r\n");
	  uart_send_bin(tx_buff, tx_buff_len);
  }
  //print_msg("transmitted \r\n");

  HAL_DCMI_Resume(&hdcmi);
}


void print_first_labels(int rows, int cols) {
	char msg [100];
    print_msg("\r\nFirst Few Labels:\r\n");
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
        	sprintf(msg, "%4d", gradientX[i][j]);
        	print_msg(msg);
        }
        print_msg("\r\n");
    }
}

void print_first_res_buff(int rows, int cols) {
	char msg [100];
    print_msg("\r\nFirst Few res_buff:\r\n");
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
        	sprintf(msg, "%4d", res_buff[i][j]);
        	print_msg(msg);
        }
        print_msg("\r\n");
    }
}

void copy(int src[3][3], int dest[3][3]){
	for (int x = 0; x < 3; x++){
		for (int y = 0; y < 3; y++){
			dest[x][y] = src[x][y];
		}
	}
}

//convert captured frame to grayscale
void convertGrayscale(){
	for (int y = 0; y < IMG_ROWS; y++){ //these for loops convert
		for (int x = 0; x < IMG_COLS; x++){
			res_buff[y][x] = (snapshot_buff[(IMG_COLS*y) + x] >> 8) & 0xFF; //reuse res_buff when computing final output
		}
	}
}

//function for Sobel Edge detection
void sobelFilter(uint8_t sourceArr[IMG_ROWS][IMG_COLS], int8_t filteredArr[IMG_ROWS][IMG_COLS], int depth, int dx, int dy, int kernelSize){
	//depth --> how much room we have to represent gradient values
	//borderType --> how to handle pixels at edge of frame

	int sobelKernel[3][3];

	int YKernel[3][3] = { //Y-direction kernel - brightness changes from top to bottom
																{-1, -2, -1},
																{0, 0, 0},
																{1, 2, 1}
															};

	int XKernel[3][3] = { //X-direction kernel - brightness changes from left to right
																{-1, 0, 1},
																{-2, 0, 2},
																{-1, 0, 1}
															};

	if (dy == 1 && dx == 0){ //there is change in brightness vertically
		copy(YKernel, sobelKernel);
	}

	else if (dx == 1 && dy == 0){ //there is change in brightness horizontally
		copy(XKernel, sobelKernel);
	}

	int kernel = 0, pixel = 0, sum = 0;

	//loop through each pixel in frame
	for (int y = 1; y < IMG_ROWS - 1; y++){ //increment pixel in y direction - avoid edge pixels
		for (int x = 1; x < IMG_COLS - 1; x++){ //increment pixel in x direction - avoid edge pixels
			sum  = 0;
			//loop through kernel, multiply 3x3 segment of pixels with designated kernel
			for (int kernCol = 0; kernCol < 3; kernCol++){ //loop through kernel's cols
				for (int kernRow = 0; kernRow < 3; kernRow++){ //loop through kernel's rows
					pixel = sourceArr[y - 1 + kernRow][x - 1 + kernCol]; //subtract 1 from y and x to extract 3x3 region around pixel
					kernel = sobelKernel[kernRow][kernCol];
					sum += pixel * kernel;
				}
			}
			filteredArr[y][x] = sum;
		}
	}
}

//combine x-direction and y-direction filtered frames
void addWeighted(int8_t gradientX[IMG_ROWS][IMG_COLS], float alpha, int8_t gradientY[IMG_ROWS][IMG_COLS], float beta, uint8_t output[IMG_ROWS][IMG_COLS]){

	uint8_t combined = 0;
	uint8_t gxPixel = 0, gyPixel = 0;
	for (int y = 0; y < IMG_ROWS; y++){
		for (int x = 0; x < IMG_COLS; x++){ //loop through combined frame and combine pixels from both frames
			gxPixel = (uint8_t)abs(gradientX[y][x]);
			gyPixel = (uint8_t)abs(gradientY[y][x]);
			combined = (alpha*gxPixel) + (beta*gyPixel);
			output[y][x] = combined;
		}
	}
}

void print_sobel(){
	const char *preamble = "\r\nPREAMBLE!\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)preamble, strlen(preamble), HAL_MAX_DELAY);
	for (int y = 0; y < IMG_ROWS; y++){
		for (int x = 0; x < IMG_COLS; x++){
			HAL_UART_Transmit(&huart3, &res_buff[y][x], 1, HAL_MAX_DELAY);
		}
	}
}

void globalThreshold(){
	uint8_t threshold = 20;
	for (int y = 0; y < IMG_ROWS; y++){
		for (int x = 0; x < IMG_COLS; x++){
			res_buff[y][x] = (res_buff[y][x] > threshold) ? 255 : 0; //if pixel intensity is above threshold, assign 255; if below threshold, assign 0
		}
	}
}

void motion_detect() {
	//print_msg("starting motion detect\r\n");
    if (start_time == 0) {
        start_time = HAL_GetTick();
    }

    if (HAL_GetTick() - start_time > WARM_UP_TIME) {
        if (dma_flag) {
            HAL_DCMI_Suspend(&hdcmi);

            // Capture the new frame into the first half of snapshot_buff
            ov7670_capture(&snapshot_buff[0]);
            //print_msg("capturing image");

            uint32_t diff = 0;
            for (int i = 0; i < HALF_FRAME_SIZE; i++) {
                uint8_t pixeldiff = (snapshot_buff[i] > snapshot_buff[i + HALF_FRAME_SIZE]) ?
                                     snapshot_buff[i] - snapshot_buff[i + HALF_FRAME_SIZE] : snapshot_buff[i + HALF_FRAME_SIZE] - snapshot_buff[i];
                diff += pixeldiff;
            }

            uint32_t framediff = (diff > old_diff) ? diff - old_diff : old_diff - diff;
            char msg[100];
            snprintf(msg, sizeof(msg), "Framediff: %lu\r\n", framediff);
            print_msg(msg);

            if (framediff > MOTION_DETECTED_THRESHOLD) {
                if (!motion_detected) {
                	HAL_Delay(1000);
                    motion_detected = 1;
                    last_motion_time = HAL_GetTick();
                    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

            ov7670_capture(snapshot_buff);
            convertGrayscale();
            sobelFilter(res_buff, gradientX, 8, 1, 0, 3);
            sobelFilter(res_buff, gradientY, 8, 0, 1, 3);
           addWeighted(gradientX, 0.5, gradientY, 0.5, res_buff);
           globalThreshold();
            //print_sobel();
                }
            }
            else if (framediff < MOTION_CLEARED_THRESHOLD && (HAL_GetTick() - last_motion_time) > MOTION_DETECTED_TIME) {
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            }
            old_diff = diff;
            memcpy(&snapshot_buff[HALF_FRAME_SIZE], &snapshot_buff[0], HALF_FRAME_SIZE * sizeof(uint16_t));

            HAL_DCMI_Resume(&hdcmi);
            dma_flag = 0;
        }
    }
}


char returnPressedKey(void){

	if (current_col == 3){

		if (HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET){ //HAL_GPIO_ReadPin checks state of PIN

			printout("0");
			returnKey = '0';

		}

		else if (HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET){

			printout("4");
			returnKey = '4';
			sprintf(buffer, "current col: %d\r\n", current_col);
			//printout(buffer);

		}

		else if (HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET){

			printout("8");
			returnKey = '8';
			sprintf(buffer, "current column: %d\r\n", current_col);
			//printout(buffer);

		}

		else if (HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET){

			printout("C");
			returnKey = 'C';

		}

	}

	else if (current_col == 2){

		if (HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET){

			printout("1");
			returnKey = '1';

		}

		else if (HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET){

			printout("5");
			returnKey = '5';

		}

		else if (HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET){

			printout("9");
			returnKey = '9';

		}

		else if (HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET){

			printout("D");
			returnKey = 'D';

		}

	}



	 else if (current_col == 1){

		if (HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET){

			printout("2");
			returnKey = '2';

		}

		else if (HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET){

			printout("6");
			returnKey = '6';

		}



	 if (HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET){

			printout("A");
		  returnKey = 'A';

		}

		else if (HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET){

			printout("E");
			returnKey = 'E';
		}

	}



	else if (current_col == 0){

		if (HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET){

			printout("3");
			returnKey = '3';

		}


		else if (HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET){

			printout("7");
			returnKey = '7';

		}

		else if (HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET){

			printout("B");
			returnKey = 'B';

		}

		else if (HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET){

			printout("F");
			returnKey = 'F';

		}

	}

	else if (current_col == -1) {

		sprintf(buffer, "current column: %d\r\n", current_col);
		//printout(buffer);

		returnKey = 'Z'; //if current_col = -1 (i.e. no button pressed)

	}

	sprintf(buffer, "wasPressed: %d\r\n", wasPressed);
	//printout(buffer);
	sprintf(buffer, "returnKey: %c\r\n", returnKey);
	//printout(buffer);

	if ((returnKey != 'Z') && (!wasPressed)) {	//if returnKey is an actual keypad value and the key value was not previously returned

		//print_msg("entered return condition\r\n");
		wasPressed = 1;
		sprintf(buffer, "key pressed: %c\r\n", returnKey);
		//printout(buffer);
		current_col = -1;
		return returnKey;

	}

	else if ((returnKey != 'Z') && (wasPressed)) { //if the same keypad value has already been returned

		current_col = -1;
		return 'Z';

	}

	else if ((returnKey == 'Z')){ //if no keys are currently pressed, reset the "has been pressed while being held" tracker

		wasPressed = 0;
		//sprintf(buffer, "%c", returnKey);
		//printout(buffer);
		current_col = -1;
		return returnKey;

	}

}

void KeyPadFSM(char digit){

	static states currentState = IDLE;

	sprintf(buffer, "state: %d\r\n", currentState);
	//printout(buffer);

	sprintf(buffer, "digit: %c\r\n", digit);
	//printout(buffer);

	if (digit == 'Z' && (currentState != VALIDPIN) && (currentState != ACCEPTED)) return; //don't do anything if no button pressed

	switch(currentState) {

		case IDLE:

			if (digit == PIN[0]) currentState = S1;

			else currentState = IDLE;

			break;

		case S1:

			if (digit == PIN[1]) currentState = S2;

			else currentState = IDLE;

			break;

		case S2:

			if (digit == PIN[2]) currentState = S3;

			else currentState = IDLE;

			break;

		case S3:

			if (digit == PIN[3]) currentState = VALIDPIN;

			else currentState = IDLE;

			print_msg("\r\nPIN VALID\r\n ACCESS GRANTED\r\n");
			pinValid = 1;

			break;

		case VALIDPIN:

			//print_msg("entered correct state 2\r\n");
			//print_msg("PIN VALID");

			/*
			while (!faceRecognized){

				HAL_Delay(1000);
				print_msg("waiting for facial recognition");

				//add RTC countdown code

			}
			*/

			currentState = ACCEPTED;

			break;

		case ACCEPTED:

			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			HAL_Delay(1000);

			currentState = IDLE;

			break;

		if(currentState == VALIDPIN) currentState = ACCEPTED;

		default: currentState = IDLE;

	}

}

void sweepRows(void){

	current_row++;
	HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);
	digit = returnPressedKey();
	KeyPadFSM(digit); //update KeyPadFSM with
	HAL_Delay(300);

	current_row++;
	HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);
	digit = returnPressedKey();
	KeyPadFSM(digit); //update KeyPadFSM with
	HAL_Delay(300);

	current_row++;
	HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);
	digit = returnPressedKey();
	KeyPadFSM(digit); //update KeyPadFSM with
	HAL_Delay(300);

	current_row++;
	HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_SET);
	digit = returnPressedKey();
	KeyPadFSM(digit); //update KeyPadFSM with
	HAL_Delay(300);

	current_row = -1;

	//print_msg("sweep completed\r\n");

}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  print_msg("hello world\r\n");
    //ov7670_init();
	if (ov7670_init() != 0) {
			  print_msg("Error: OV7670 initialization failed!\r\n");
	}
    ov7670_capture(snapshot_buff);

//    print_msg("captured!\r\n");

  //ov7670_init();
  //HAL_Delay(2000);
//  if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin)) {
//	  srand(HAL_GetTick());
//	  send_random_code();
//  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  motion_detect();//print_msg("entered while loop\r\n");
	  //ov7670_snapshot(snapshot_buff);
	  //motion_detect();
//	  if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin)) {			//chnage this to if motion detected
//	  	    HAL_Delay(400);  // debounce
//	  	    print_msg("Snap!\r\n");
//	  	    ov7670_snapshot(snapshot_buff); // Capture image, store into snapshot buffer
//	  	  	print_msg("Success!\n");
//
//	  	    convertGrayscale(); //convert snapshot buffer to grayscale
//			print_msg("Converted to grayscale\r\n");
//			sobelFilter(res_buff, gradientX, 8, 1, 0, 3);
//			sobelFilter(res_buff, gradientY, 8, 0, 1, 3);
//			addWeighted(gradientX, 0.5, gradientY, 0.5, res_buff);
//			globalThreshold();
//
//			print_sobel();
	  	  	 if (motion_detected == 1){

	  	  		//motion_detected = 0;
	  	  		 HAL_Delay(1000);
					memset(snapshot_buff, 0, MAX_LABELS * sizeof(uint16_t));
					uint16_t* equivalenceTable = (uint16_t*)snapshot_buff;
					if (equivalenceTable == NULL) {
						print_msg("Error: Equivalence table memory allocation failed\r\n");
						return;
					}

					initEquivalenceTable(equivalenceTable);
					//print_first_res_buff(144, 174);
					firstPass();
					print_first_labels(144, 174);
					detectHuman();
					if (humanDetected){
						srand(HAL_GetTick());
						send_random_code();
						while (pinValid == 0 ){
							sweepRows();
							KeyPadFSM(digit); //update KeyPadFSM with
							HAL_Delay(20);
							}
							pinValid = 0;
						humanDetected = 0;
					}
					HAL_Delay(50);
					motion_detected =0;
					//print_msg("exitng human detect\r\n");



					//motion_detected = 0;
	  	  	 }


			//print_labels();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

//void MX_DMA_Init(void)
//{
//
//  /* DMA controller clock enable */
//  HAL_RCC_DMA2_CLK_ENABLE();
//  HAL_RCC_DMA1_CLK_ENABLE();
//
//  // DMA interrupt init
//  // DMA1_Stream3_IRQn interrupt configuration
//  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
//  // DMA2_Stream1_IRQn interrupt configuration
//  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
//
//}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ROW2_Pin|ROW3_Pin|ROW0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW2_Pin ROW3_Pin ROW0_Pin */
  GPIO_InitStruct.Pin = ROW2_Pin|ROW3_Pin|ROW0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW1_Pin */
  GPIO_InitStruct.Pin = ROW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ROW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COL0_Pin COL1_Pin COL2_Pin COL3_Pin */
  GPIO_InitStruct.Pin = COL0_Pin|COL1_Pin|COL2_Pin|COL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
