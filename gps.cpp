#include "gps.hpp"

#include <iostream>

GPS::GPS() : LED_PIN(PICO_DEFAULT_LED_PIN) { }

bool GPS::read_line(char* buffer, size_t buffer_size, uart_inst_t *uart_port) {
    char character;
    int index = 0;

    constexpr char end = 10;

    while (index < buffer_size - 1) {
        character = uart_getc(uart_port);

        if (character == 0) {
            continue;
        }

        buffer[index] = character;

        index = index + 1;

        if (character == end) {
            buffer[index] = 0;
            return true;
        }
    }

    buffer[index] = 0;
    return false;
}

void GPS::init () {
    uart_init(uart0, 9600);
    
    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

void GPS::read (struct minmea_sentence_gll* output) {
    char line[MINMEA_MAX_LENGTH];

    bool no_gps_output_yet = true;
    while (no_gps_output_yet) {
        if (read_line(line, MINMEA_MAX_LENGTH, uart0)) {
            switch (minmea_sentence_id(line, false)) {
                case MINMEA_SENTENCE_GLL: {
                    if (minmea_parse_gll(output, line)) {
                        no_gps_output_yet = false;
                    }
                } break;
                /*case MINMEA_SENTENCE_GSV: {
                    struct minmea_sentence_gsv frame;
                    if (minmea_parse_gsv(&frame, line)) {
                        std::cout << "$GSV: sattelites in view: " << frame.total_sats << std::endl;
                    }
                } break;*/
            }
        }
    }
}
