#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <stdbool.h>

// http://nic.vajn.icu/PDF/STMicro/ARM/STM32F0/STM32F0xx_IR_remote.pdf
// ------------------ SIRC ------------------
// Time unit is 10µs (microseconds)
#define START_BIT_LOW 200
#define START_BIT_HIGH 280
#define ONE_BIT_LOW 100
#define ONE_BIT_HIGH 140
#define ZERO_BIT_LOW 40
#define ZERO_BIT_HIGH 80
#define START_BLOCK_HIGH 360
#define START_BLOCK_LOW 240
#define ONE_BLOCK_LOW 140
#define ONE_BLOCK_HIGH 220
#define ZERO_BLOCK_LOW 90
#define ZERO_BLOCK_HIGH 150
#define SIRC_LENGTH 12

// ------------------ Licznik ------------------
#define SEKUND_5 5000 
#define SEKUND_10 10000
#define ONE_MILISECOND_TIMER 15999
#define TEN_MICROSECONDS_TIMER 159
#define NINETY_MILISECONDS_ARR 90
// ------------------ Diody ------------------

#define RED_LED_GPIO GPIOA
#define GREEN_LED_GPIO GPIOA
#define BLUE_LED_GPIO GPIOB
#define GREEN2_LED_GPIO GPIOA
#define RED_LED_PIN 6
#define GREEN_LED_PIN 7
#define BLUE_LED_PIN 0
#define GREEN2_LED_PIN 5

#define RedLEDon() \
    RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16)
#define RedLEDoff() \
    RED_LED_GPIO->BSRR = 1 << RED_LED_PIN

#define GreenLEDon() \
    GREEN_LED_GPIO->BSRR = 1 << (GREEN_LED_PIN + 16)
#define GreenLEDoff() \
    GREEN_LED_GPIO->BSRR = 1 << GREEN_LED_PIN

#define BlueLEDon() \
    BLUE_LED_GPIO->BSRR = 1 << (BLUE_LED_PIN + 16)
#define BlueLEDoff() \
    BLUE_LED_GPIO->BSRR = 1 << BLUE_LED_PIN

#define Green2LEDon() \
    GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN
#define Green2LEDoff() \
    GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16)

// ------------------ PRZYCISKI ------------------
// ------------------ AKTYWNY 0 ------------------

#define USER_BTN_GPIO GPIOC
#define USER_BTN_PIN 13

#define LEFT_BTN_GPIO GPIOB
#define LEFT_BTN_PIN 3
#define RIGHT_BTN_GPIO GPIOB
#define RIGHT_BTN_PIN 4
#define UP_BTN_GPIO GPIOB
#define UP_BTN_PIN 5
#define DOWN_BTN_GPIO GPIOB
#define DOWN_BTN_PIN 6
#define ACTION_BTN_GPIO GPIOB
#define ACTION_BTN_PIN 10

// ------------------ AKTYWNY 1 ------------------

#define AT_BTN_GPIO GPIOA
#define AT_BTN_PIN 0

// ------------------ USART ------------------

#define USART_Mode_Rx_Tx (USART_CR1_RE | USART_CR1_TE)
#define USART_Enable USART_CR1_UE
#define USART_WordLength_8b 0x0000
#define USART_Parity_No 0x0000
#define USART_FlowControl_None 0x0000
#define USART_StopBits_1 0x0000     // Liczba bitów przerwy między komunikatami.
#define HSI_HZ 16000000U
#define PCLK1_HZ HSI_HZ             // Zmienne do ustalenia prędkości przesyłania danych.
#define BAUD 9600U

// ------------------ USART ------------------

// ------------------ RECEIVER ------------------

#define RECEIVER_GPIO GPIOA
#define RECEIVER_PIN 14

// ------------------ RECEIVER ------------------


#define BUFFOR_SIZE 10000

struct cyclic_buffer {
    char* start;
    char* end;
    char* head;
    char* tail;
    unsigned size;
};

typedef struct cyclic_buffer cyclic_buffer;


cyclic_buffer c;

void init(cyclic_buffer* c) {
    static char storage[BUFFOR_SIZE];  
    c->size = BUFFOR_SIZE;
    c->start = storage;
    c->end = c->start + c->size;
    c->head = c->start;
    c->tail = c->start;
}

bool is_empty(cyclic_buffer* c) {
    if(c->tail == c->head) {
        return true;
    }
    return false;
}

char* next_in_buffer(cyclic_buffer* c, char* pointer) {
    if(pointer + 1 == c->end) {
        return c->start;
    }
    return pointer + 1;
}

void push_back(cyclic_buffer* c, const char* s) {
    while(*s != '\0') {
        *(c->head) = *s;
        s++;
        char* next = next_in_buffer(c, c->head);
        if(next == c->tail) {
            break;
        }
        c->head = next;
    }
}

char pop_front(cyclic_buffer* c) {
    char rez = *(c->tail);
    c->tail = next_in_buffer(c, c->tail);
    return rez;
}

char* get_tail(cyclic_buffer* c) {
    return c->tail;
}

unsigned get_first_message_length(cyclic_buffer* c) {
    char* find_end = c->tail;
    unsigned length = 1;
    while(*find_end != '\n') {
        find_end = next_in_buffer(c, find_end);
        length++;
    }
    return length;
}

void pop_front_x_bytes(cyclic_buffer* c, unsigned pop_n) {
    for(unsigned i = 0; i < pop_n; i++) {
        pop_front(c);
    }
}

void left_pressed(cyclic_buffer* c) {
    if((LEFT_BTN_GPIO->IDR >> LEFT_BTN_PIN) & 1) {
        push_back(c, "LEFT RELEASED\r\n");
    }
    else {
        push_back(c, "LEFT PRESSED\r\n");
    }
}

void right_pressed(cyclic_buffer* c) {
    if((RIGHT_BTN_GPIO->IDR >> RIGHT_BTN_PIN) & 1) {
        push_back(c, "RIGHT RELEASED\r\n");
    }
    else {
        push_back(c, "RIGHT PRESSED\r\n");
    }
}

void up_pressed(cyclic_buffer* c) {
    if((UP_BTN_GPIO->IDR >> UP_BTN_PIN) & 1) {
        push_back(c, "UP RELEASED\r\n");
    }
    else {
        push_back(c, "UP PRESSED\r\n");
    }
}

void down_pressed(cyclic_buffer* c) {
    if((DOWN_BTN_GPIO->IDR >> DOWN_BTN_PIN) & 1) {
        push_back(c, "DOWN RELEASED\r\n");
    }
    else {
        push_back(c, "DOWN PRESSED\r\n");
    }
  
}

void action_pressed(cyclic_buffer* c) {
    if((ACTION_BTN_GPIO->IDR >> ACTION_BTN_PIN) & 1) {
        push_back(c, "FIRE RELEASED\r\n");
    }
    else {
        push_back(c, "FIRE PRESSEDD\r\n");
    }
}

void user_pressed(cyclic_buffer* c) {
    if((USER_BTN_GPIO->IDR >> USER_BTN_PIN) & 1) {
        push_back(c, "USER RELEASED\r\n");
    }
    else {
        push_back(c, "USER PRESSED\r\n");
    }
}

void mode_pressed(cyclic_buffer* c) {
    if(!((AT_BTN_GPIO->IDR >> AT_BTN_PIN) & 1)) {
        push_back(c, "MODE RELEASED\r\n");
    }
    else {
        push_back(c, "MODE PRESSED\r\n");
    }
}

void sygnal_received(cyclic_buffer* c) {
    push_back(c, "SIGNAL RECEIVED\r\n");
}

void send_message(cyclic_buffer* c) {
    DMA1_Stream6->M0AR = (uint32_t)get_tail(c);
    DMA1_Stream6->NDTR = get_first_message_length(c);
    DMA1_Stream6->CR |= DMA_SxCR_EN;
    
}

// Funkcja ta wywołuje się asynchronicznie w stosunku do mojego programu, nie mogą jej wywoływać.
// Funkcja ta wywoluje sie gdy nastepuje przerwanie.
void DMA1_Stream6_IRQHandler() { 
    uint32_t isr = DMA1->HISR;          // Interrupt Status Register, ustawiony na 1 jeżeli przesłanie zakończyło się sukcesem
    if (isr & DMA_HISR_TCIF6) {
        pop_front_x_bytes(&c, get_first_message_length(&c));

        DMA1->HIFCR = DMA_HIFCR_CTCIF6;         //Interrput Fluck Clear Register -> zeruje HISR.
        
        if(!is_empty(&c)) {
            send_message(&c);
        }
    }
}

void check_if_available() {
    // ten warunek sprawdać gdy przycisk zgłasza przerwania i jeżeli DMA nie jest zajęte to od razu
    // dajemy do DMA a w.p.p kolejkujemy.
    if((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 && (DMA1->HISR & DMA_HISR_TCIF6) == 0) {
        send_message(&c);
    }
}


void EXTI3_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR3;
    left_pressed(&c);
    check_if_available();
}

void EXTI4_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR4;
    right_pressed(&c);
    check_if_available();
}

void EXTI9_5_IRQHandler(void) {
    if(EXTI->PR & EXTI_PR_PR5) {
        EXTI->PR = EXTI_PR_PR5;
        up_pressed(&c);
    }
    else {
        EXTI->PR = EXTI_PR_PR6;
        down_pressed(&c);
    }

    check_if_available();
}

char* utoa_dec(char *p, uint16_t v)
{
    char tmp[6];
    int i = 0;

    do {
        tmp[i++] = '0' + (v % 10);
        v /= 10;
    } while (v);

    while (i--)
        *p++ = tmp[i];

    return p;
}

// Mierzę czasy zbocza opadającego, ile ono trwało, czyli fragmenty między dwoma sąsiednimi stanami wysokimi.
// Mierzę także czas całego "bit bloku", czyli czas między kolejnymi zboczami opadającymi.
uint16_t frame = 0;
uint8_t bit_cnt = 0;
bool transmission_started = false;
uint16_t last_low_state_time = 0;        // Czas od ostatniego startu stanu niskiego.
uint16_t last_low_block_duration = 0;   // Czas przez jaki trwał ostatni stan niski.
volatile bool data_ready = false;
volatile uint16_t string_frame = 0;

void reset_values() {
    bit_cnt = 0;
    frame = 0;
    transmission_started = false;
    last_low_block_duration = 0;
    last_low_state_time = 0;
    RedLEDoff();
}

void EXTI15_10_IRQHandler(void) {
    if(EXTI->PR & EXTI_PR_PR14) {
        // sygnal_received(&c);
        // check_if_available();
        EXTI->PR = EXTI_PR_PR14; // Wyczyszczenie flagi przerwania

        // Przerwanie wywołane zboczem opadającym.
        if (!((RECEIVER_GPIO->IDR & (1 << RECEIVER_PIN)))) {
            // Początek transmisji, pierwszy stan niski.
            if(!transmission_started && last_low_state_time == 0) {
                last_low_state_time = TIM3->CNT;
            }   // Drugi stan niski od początku transmisji, sprawdzam czy czas to mniej więcej 3000µs.
            else if(!transmission_started && last_low_state_time != 0) {
                uint16_t block_time = (uint16_t)(TIM3->CNT - last_low_state_time);
                // Bit startowy zaistniał
                if((block_time > START_BLOCK_LOW && block_time < START_BLOCK_HIGH) && (last_low_block_duration > START_BIT_LOW && last_low_block_duration < START_BIT_HIGH)) {
                    RedLEDon();
                    transmission_started = true;
                    last_low_state_time = TIM3->CNT;
                }   // Zły sygnał.
                else {
                    reset_values();
                }
            } // Kolejne stany niskie w transmiji, sprawdzam czy jednynka, zero, czy błąd.
            else if(transmission_started) {
                uint16_t block_time = (uint16_t)(TIM3->CNT - last_low_state_time);
                last_low_state_time = TIM3->CNT;
                if((block_time > ONE_BLOCK_LOW && block_time < ONE_BLOCK_HIGH) && (last_low_block_duration > ONE_BIT_LOW && last_low_block_duration < ONE_BIT_HIGH)) {
                    frame |= (1 << bit_cnt);
                    bit_cnt++;
                }
                else if((block_time > ZERO_BLOCK_LOW && block_time < ZERO_BLOCK_HIGH) && (last_low_block_duration > ZERO_BIT_LOW && last_low_block_duration < ZERO_BIT_HIGH)) {
                    bit_cnt++;
                }
                else {
                    reset_values();
                }
            }
        }   
        else {  // Przerwanie wywołane zboczem rosnącym.
            // Zaistniał stan niski, zapamiętuję jego czas.
            if(transmission_started || last_low_state_time != 0) {
                last_low_block_duration = TIM3->CNT - last_low_state_time;
            }
            // Protokół został w całości przesłany.
            if(bit_cnt == SIRC_LENGTH - 1) {
                if(last_low_block_duration > ONE_BIT_LOW && last_low_block_duration < ONE_BIT_HIGH) {
                    frame |= (1 << bit_cnt);
                }
                bit_cnt++;
                string_frame = frame;
                data_ready = true;
                reset_values();
            }
        }
    }
}

int main() {

    init(&c);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
    RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA1EN;

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // LICZNIK
    // Włączenie taktowania licznika.
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    __NOP();
    RedLEDoff();
    GreenLEDoff();
    BlueLEDoff();
    Green2LEDoff();

    GPIOoutConfigure(RED_LED_GPIO,
        RED_LED_PIN,
        GPIO_OType_PP,
        GPIO_Low_Speed,
        GPIO_PuPd_NOPULL);

    GPIOoutConfigure(GREEN_LED_GPIO,
        GREEN_LED_PIN,
        GPIO_OType_PP,
        GPIO_Low_Speed,
        GPIO_PuPd_NOPULL);
    
    GPIOoutConfigure(BLUE_LED_GPIO,
        BLUE_LED_PIN,
        GPIO_OType_PP,
        GPIO_Low_Speed,
        GPIO_PuPd_NOPULL);
    
    GPIOoutConfigure(GREEN2_LED_GPIO,
        GREEN2_LED_PIN,
        GPIO_OType_PP,
        GPIO_Low_Speed,
        GPIO_PuPd_NOPULL);


    GPIOafConfigure(GPIOA,
        2,
        GPIO_OType_PP,
        GPIO_Fast_Speed,
        GPIO_PuPd_NOPULL,
        GPIO_AF_USART2);
    
    GPIOafConfigure(GPIOA,
        3,
        GPIO_OType_PP,
        GPIO_Fast_Speed,
        GPIO_PuPd_UP,
        GPIO_AF_USART2); 
    
    // Trzeba skonfigurować przerwania dla odpowiedniego układu i linii na które jest guzik.
    // Co jeżeli zrobię najpierw falling a potem Rising?
    GPIOinConfigure(RECEIVER_GPIO, RECEIVER_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    GPIOinConfigure(LEFT_BTN_GPIO, LEFT_BTN_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    GPIOinConfigure(RIGHT_BTN_GPIO, RIGHT_BTN_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    GPIOinConfigure(DOWN_BTN_GPIO, DOWN_BTN_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
    GPIOinConfigure(UP_BTN_GPIO, UP_BTN_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);

    USART2->CR1 = USART_Mode_Rx_Tx | USART_WordLength_8b | USART_Parity_No;
    USART2->CR2 = USART_StopBits_1;
    USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
    USART2->BRR = (PCLK1_HZ + (BAUD / 2U)) / BAUD;

    DMA1_Stream6->CR = 4U << 25 |
        DMA_SxCR_PL_1 |
        DMA_SxCR_MINC |
        DMA_SxCR_DIR_0 |
        DMA_SxCR_TCIE;

    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

    DMA1_Stream5->CR = 4U << 25 |
        DMA_SxCR_PL_1 |
        DMA_SxCR_MINC |
        DMA_SxCR_TCIE;
    
    DMA1_Stream5->PAR = (uint32_t)&USART2->DR;

    DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTCIF5;

    // Zerujemy bity przerwań, które mogły się zapalić przy odpalaniu programu (ich stan nie jest znany).
    EXTI->PR = EXTI_PR_PR14;

    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    

    USART2->CR1 |= USART_Enable;        // włączenie UARTU na koniec.

 
    TIM3->PSC = TEN_MICROSECONDS_TIMER; // 159 -> 16MHz/160 = 100kHz (1 cykl = 10us)
    TIM3->ARR = 0xFFFF;         // Maksymalna wartość, nie interesuja mnie przerwania zegara.
    TIM3->CR1 = 0;              // Wyłączony na start
    TIM3->CNT = 0;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;

    TIM3->CR1 |= TIM_CR1_CEN;
    
    while(1) {
        if(data_ready) {
            uint8_t command = string_frame & 0x7F;          
            uint8_t address = (string_frame >> 7) & 0x1F;

            char tx_buf[32];
            char *p = tx_buf;

            *p++ = 'C'; *p++ = 'M'; *p++ = 'D'; *p++ = '=';
            p = utoa_dec(p, command);
            *p++ = ' ';
            *p++ = 'A'; *p++ = 'D'; *p++ = 'D'; *p++ = 'R'; *p++ = '=';
            p = utoa_dec(p, address);
            *p++ = '\r'; *p++ = '\n'; *p = '\0';
            
            push_back(&c, tx_buf);
            check_if_available();
            data_ready = false;
        }
    }

    return 0;
}