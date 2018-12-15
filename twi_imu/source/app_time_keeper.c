#include "app_time_keeper.h"
#include "counter.h"

static uint16_t m_counter_freq; //Hz

static float m_deltat = 0.0f;      // integration interval for both filter schemes
static uint32_t m_count_last = 0; // used to calculate integration interval
static uint32_t m_count_now = 0;    // used to calculate integration interval

void init_time_keeper(uint16_t freq)
{
    m_counter_freq = freq;
    counter_init(m_counter_freq);
    counter_start();
}

float update_time(void)
{

    m_count_now = counter_get();

    // Set integration time by time elapsed since last filter update
    if(m_count_now >= m_count_last)
    {
        m_deltat = (float)(m_count_now - m_count_last) / m_counter_freq;
    }
    else //Counter overflow
    {
        m_deltat = (float)(0xFFFFFF - m_count_last + m_count_now) / m_counter_freq;
    }

    m_count_last = m_count_now;

    return m_deltat;
}
