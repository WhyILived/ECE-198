#include "stm32f401.h"

#define I2C1_BASE      0x40005400
#define I2C1_REG       ((I2C_TypeDef *) I2C1_BASE)
#define BMP280_ADDR    0x76

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_TypeDef;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

#define USART2_BASE    0x40004400
#define USART2_REG     ((USART_TypeDef *) USART2_BASE)

void GPIO_Init(void);
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t data);
uint8_t I2C_Read_Ack(void);
uint8_t I2C_Read_Nack(void);
void I2C_WaitForFlag(uint32_t flag);
void BMP280_Init(void);
void BMP280_Read_Raw_Data(uint8_t reg, uint8_t *data, uint8_t len);
void BMP280_Read_Temperature_Pressure(int32_t *temperature, int32_t *pressure);
void UART_Init(void);
void UART_Send(char c);
void UART_Send_String(char *str);
void UART_Send_Number(int32_t num);

int main(void) {
    GPIO_Init();
    I2C_Init();
    UART_Init();
    BMP280_Init();
    int32_t temperature = 0, pressure = 0;
    char buffer[50];
    while(1) {
        BMP280_Read_Temperature_Pressure(&temperature, &pressure);
        UART_Send_String("Temperature: ");
        UART_Send_Number(temperature);
        UART_Send_String(" C\r\n");
        UART_Send_String("Pressure: ");
        UART_Send_Number(pressure);
        UART_Send_String(" Pa\r\n");
        for(int i=0;i<1000000;i++);
    }
}

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOB->MODER |= (0x2 << (8 * 2)) | (0x2 << (9 * 2));
    GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOB->AFR[1] |= (0x4 << 0) | (0x4 << 4);
    GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
    GPIOB->PUPDR |= (0x1 << (8 * 2)) | (0x1 << (9 * 2));
    GPIOB->OSPEEDR |= (0x3 << (8 * 2)) | (0x3 << (9 * 2));
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (0x2 << (2 * 2)) | (0x2 << (3 * 2));
    GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12));
    GPIOA->AFR[0] |= (0x7 << 8) | (0x7 << 12);
    GPIOA->OTYPER |= GPIO_OTYPER_OT_2 | GPIO_OTYPER_OT_3;
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3);
    GPIOA->PUPDR |= (0x1 << (2 * 2)) | (0x1 << (3 * 2));
    GPIOA->OSPEEDR |= (0x3 << (2 * 2)) | (0x3 << (3 * 2));
}

void I2C_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1_REG->CR1 &= ~0x1;
    I2C1_REG->CR2 |= 16;
    I2C1_REG->CCR = 80;
    I2C1_REG->TRISE = 17;
    I2C1_REG->CR1 |= 0x1;
}

void I2C_Start(void) {
    I2C1_REG->CR1 |= 0x100;
    I2C_WaitForFlag(0x1);
}

void I2C_Stop(void) {
    I2C1_REG->CR1 |= 0x200;
}

void I2C_Write(uint8_t data) {
    I2C_WaitForFlag(0x80);
    I2C1_REG->DR = data;
}

uint8_t I2C_Read_Ack(void) {
    uint8_t data;
    I2C1_REG->CR1 |= 0x4;
    I2C_WaitForFlag(0x40);
    data = I2C1_REG->DR;
    return data;
}

uint8_t I2C_Read_Nack(void) {
    uint8_t data;
    I2C1_REG->CR1 &= ~0x4;
    I2C1_REG->CR1 |= 0x200;
    I2C_WaitForFlag(0x40);
    data = I2C1_REG->DR;
    return data;
}

void I2C_WaitForFlag(uint32_t flag) {
    while(!(I2C1_REG->SR1 & flag));
}

void BMP280_Init(void) {
    I2C_Start();
    I2C_Write((BMP280_ADDR << 1) | 0);
    I2C_Write(0xF2);
    I2C_Write(0x01);
    I2C_Stop();
    I2C_Start();
    I2C_Write((BMP280_ADDR << 1) | 0);
    I2C_Write(0xF4);
    I2C_Write(0x27);
    I2C_Stop();
    I2C_Start();
    I2C_Write((BMP280_ADDR << 1) | 0);
    I2C_Write(0xF5);
    I2C_Write(0xA0);
    I2C_Stop();
}

void BMP280_Read_Raw_Data(uint8_t reg, uint8_t *data, uint8_t len) {
    I2C_Start();
    I2C_Write((BMP280_ADDR << 1) | 0);
    I2C_Write(reg);
    I2C_Stop();
    I2C_Start();
    I2C_Write((BMP280_ADDR << 1) | 1);
    for(uint8_t i=0;i<len-1;i++) {
        data[i] = I2C_Read_Ack();
    }
    data[len-1] = I2C_Read_Nack();
    I2C_Stop();
}

void BMP280_Read_Temperature_Pressure(int32_t *temperature, int32_t *pressure) {
    uint8_t data[6];
    BMP280_Read_Raw_Data(0xF7, data, 6);
    int32_t adc_p = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    int32_t adc_t = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
    *pressure = adc_p;
    *temperature = adc_t;
}

void UART_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2_REG->CR1 = 0x00000000;
    USART2_REG->BRR = 0x0683;
    USART2_REG->CR1 |= 0x200C;
    USART2_REG->CR2 = 0x0000;
    USART2_REG->CR3 = 0x0000;
}

void UART_Send(char c) {
    while(!(USART2_REG->SR & 0x80));
    USART2_REG->DR = c;
}

void UART_Send_String(char *str) {
    while(*str) {
        UART_Send(*str++);
    }
}

void UART_Send_Number(int32_t num) {
    char buffer[12];
    int i=0;
    int isNegative = 0;
    if(num == 0){
        UART_Send('0');
        return;
    }
    if(num < 0){
        isNegative = 1;
        num = -num;
    }
    while(num > 0){
        buffer[i++] = (num % 10) + '0';
        num /=10;
    }
    if(isNegative){
        UART_Send('-');
    }
    while(i--){
        UART_Send(buffer[i]);
    }
}