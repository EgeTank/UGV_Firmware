// encoder_reader.h
#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <stdint.h>


float encoder_calculate_speed_mm_s(int16_t delta_count, float delta_time_s);

#endif /* ENCODER_READER_H */
