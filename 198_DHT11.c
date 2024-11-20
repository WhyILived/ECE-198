#include "stm32f401.h"

#define USART2_BASE    0x40004400
#define USART2_REG     ((USART_TypeDef *) USART2_BASE)
#define DHT11_PIN      0
#define DHT11_PORT     GPIOA
#define DHT11_MODE_INPUT 0x00
#define DHT11_MODE_OUTPUT 0x01

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

void GPIO_Init(void);
void UART_Init(void);
void UART_Send(char c);
void UART_Send_String(char *str);
void UART_Send_Number(uint8_t num);
void delay_ms(uint32_t ms);
uint8_t DHT11_Read(uint8_t *humidity, uint8_t *temperature);

int main(void) {
    GPIO_Init();
    UART_Init();
    uint8_t humidity = 0, temperature = 0;
    while(1) {
        if(DHT11_Read(&humidity, &temperature)) {
            UART_Send_String("Temperature: ");
            UART_Send_Number(temperature);
            UART_Send_String(" C\r\n");
            UART_Send_String("Humidity: ");
            UART_Send_Number(humidity);
            UART_Send_String(" RH\r\n");
        }
        delay_ms(2000);
    }
}

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(0x3 << (DHT11_PIN * 2));
    GPIOA->MODER |= (0x1 << (DHT11_PIN * 2));
    GPIOA->OTYPER |= (1 << DHT11_PIN);
    GPIOA->PUPDR &= ~(0x3 << (DHT11_PIN * 2));
    GPIOA->PUPDR |= (0x1 << (DHT11_PIN * 2));
}

void UART_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~((0x3 << (2 * 2)) | (0x3 << (3 * 2)));
    GPIOA->MODER |= (0x2 << (2 * 2)) | (0x2 << (3 * 2));
    GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12));
    GPIOA->AFR[0] |= (0x7 << 8) | (0x7 << 12);
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

void UART_Send_Number(uint8_t num) {
    char buffer[4];
    int i=0;
    if(num == 0){
        UART_Send('0');
        return;
    }
    while(num > 0){
        buffer[i++] = (num % 10) + '0';
        num /=10;
    }
    while(i--){
        UART_Send(buffer[i]);
    }
}

void delay_ms(uint32_t ms) {
    for(uint32_t i=0;i<ms*4000;i++);
}

uint8_t DHT11_Read(uint8_t *humidity, uint8_t *temperature) {
    uint8_t data[5] = {0};
    GPIOA->MODER &= ~(0x3 << (DHT11_PIN * 2));
    GPIOA->MODER |= (0x1 << (DHT11_PIN * 2));
    GPIOA->ODR &= ~(1 << DHT11_PIN);
    delay_ms(20);
    GPIOA->ODR |= (1 << DHT11_PIN);
    delay_ms(1);
    GPIOA->MODER &= ~(0x3 << (DHT11_PIN * 2));
    GPIOA->PUPDR &= ~(0x3 << (DHT11_PIN * 2));
    GPIOA->PUPDR |= (0x1 << (DHT11_PIN * 2));
    for(int i=0;i<85;i++);
    for(int i=0;i<40;i++) {
        while(!(GPIOA->IDR & (1 << DHT11_PIN)));
        uint32_t t = 0;
        while(GPIOA->IDR & (1 << DHT11_PIN)){
            t++;
            if(t > 10000) break;
        }
        data[i/8] <<=1;
        if(t > 50){
            data[i/8] |=1;
        }
        for(int j=0;j<30;j++);
    }
    if(data[4] == (data[0] + data[1] + data[2] + data[3])){
        *humidity = data[0];
        *temperature = data[2];
        return 1;
    }
    return 0;
}