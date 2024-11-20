#include "stm32f4xx.h"

void I2C1_Init(void);
void I2C_Start(void);
void I2C_Write(uint8_t data);
uint8_t I2C_Read_Ack(void);
uint8_t I2C_Read_Nack(void);
void I2C_Stop(void);
void MPU6050_Init(void);
void MPU6050_Read_Accel(int16_t *ax, int16_t *ay, int16_t *az);
void Delay(volatile uint32_t delay);

#define MPU6050_ADDR 0x68 
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1  0x6B
#define ACCEL_XOUT_H 0x3B

int main(void) {
    int16_t ax, ay, az;

    I2C1_Init();
    MPU6050_Init();

    while (1) {
        MPU6050_Read_Accel(&ax, &ay, &az);
        Delay(100000);
    }
}

void I2C1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure PB8 (SCL) and PB9 (SDA) as alternate function (AF4)
    GPIOB->MODER |= (2U << GPIO_MODER_MODER8_Pos) | (2U << GPIO_MODER_MODER9_Pos);
    GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;  // Open-drain
    GPIOB->OSPEEDR |= (3U << GPIO_OSPEEDER_OSPEEDR8_Pos) | (3U << GPIO_OSPEEDER_OSPEEDR9_Pos);  // High speed
    GPIOB->PUPDR |= (1U << GPIO_PUPDR_PUPDR8_Pos) | (1U << GPIO_PUPDR_PUPDR9_Pos);  // Pull-up
    GPIOB->AFR[1] |= (4U << GPIO_AFRH_AFSEL8_Pos) | (4U << GPIO_AFRH_AFSEL9_Pos);  // AF4 for I2C1

    I2C1->CR1 = 0;  // Disable I2C1 for configuration
    I2C1->CR2 = 16; // APB1 clock frequency in MHz
    I2C1->CCR = 80; // Standard mode (100kHz) clock
    I2C1->TRISE = 17;  // Max rise time
    I2C1->CR1 |= I2C_CR1_PE;  // Enable I2C1
}

void I2C_Start(void) {
    I2C1->CR1 |= I2C_CR1_START;  // Generate START condition
    while (!(I2C1->SR1 & I2C_SR1_SB));  // Wait for START condition to be generated
}

void I2C_Write(uint8_t data) {
    I2C1->DR = data;  // Write data to data register
    while (!(I2C1->SR1 & I2C_SR1_TXE));  // Wait for data transmission
}

uint8_t I2C_Read_Ack(void) {
    I2C1->CR1 |= I2C_CR1_ACK;  // Enable ACK
    while (!(I2C1->SR1 & I2C_SR1_RXNE));  // Wait for data reception
    return I2C1->DR;
}

uint8_t I2C_Read_Nack(void) {
    I2C1->CR1 &= ~I2C_CR1_ACK;  // Disable ACK
    while (!(I2C1->SR1 & I2C_SR1_RXNE));  // Wait for data reception
    return I2C1->DR;
}

void I2C_Stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;  // Generate STOP condition
}

void MPU6050_Init(void) {
    I2C_Start();
    I2C_Write(MPU6050_ADDR << 1);  // Send slave address with write bit
    I2C_Write(PWR_MGMT_1);  // Write to power management register
    I2C_Write(0x00);  // Wake up MPU6050
    I2C_Stop();
}

void MPU6050_Read_Accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t accel_data[6];

    I2C_Start();
    I2C_Write(MPU6050_ADDR << 1);
    I2C_Write(ACCEL_XOUT_H);
    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 1); 

    for (int i = 0; i < 5; i++) {
        accel_data[i] = I2C_Read_Ack();
    }
    accel_data[5] = I2C_Read_Nack(); 

    I2C_Stop();

    *ax = (int16_t)(accel_data[0] << 8 | accel_data[1]);
    *ay = (int16_t)(accel_data[2] << 8 | accel_data[3]);
    *az = (int16_t)(accel_data[4] << 8 | accel_data[5]);
}

void Delay(volatile uint32_t delay) {
    while (delay--);
}
