#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "config.h"

enum
{
    STEER_NORMAL,
    STEER_LEFT_WALL,
    STEER_RIGHT_WALL,
    STEERING_OFF,
};

const uint8_t NO_START = 0;
const uint8_t LEFT_START = 1;
const uint8_t RIGHT_START = 2;

struct SensorChannel
{
    float raw;
    float value;
};

class Sensors;
extern Sensors sensors;

class Sensors
{
public:
    volatile SensorChannel lss;
    volatile SensorChannel rss;
    volatile SensorChannel ff;

    VL53L0X loxR;
    VL53L0X loxL;
    VL53L0X loxF;

    volatile uint16_t distL;
    volatile uint16_t distR;
    volatile uint16_t distF;

    volatile uint16_t defL = 0;
    volatile uint16_t defR = 0;

    volatile bool see_front_wall;
    volatile bool see_left_wall;
    volatile bool see_right_wall;

public:
    uint8_t g_steering_mode = STEER_NORMAL;

    int get_front_sum()
    {
        return m_front_sum;
    }
    int get_front_diff()
    {
        return int(m_front_diff);
    }
    float get_steering_feedback()
    {
        return m_steering_adjustment;
    }
    float get_cross_track_error()
    {
        return m_cross_track_error;
    }

    void I2C_INIT()
    {
        pinMode(X_SHUTF, OUTPUT);
        pinMode(X_SHUTR, OUTPUT);
        pinMode(X_SHUTL, OUTPUT);

        digitalWrite(X_SHUTF, LOW);
        digitalWrite(X_SHUTR, LOW);
        digitalWrite(X_SHUTL, LOW);
        delay(10);

        BTSerial.println("Khoi dong Cam bien R...");
        digitalWrite(X_SHUTR, HIGH);
        delay(10);
        if (!loxR.init())
        {
            BTSerial.println(F("Loi khoi tao Cam bien R!"));
            while (1)
                ;
        }
        loxR.setAddress(ADDRR);
        BTSerial.print("Cam bien R doi sang 0x");
        BTSerial.println(ADDRR, HEX);
        loxR.setMeasurementTimingBudget(20000);
        loxR.startContinuous();

        BTSerial.println("Khoi dong Cam bien F...");
        digitalWrite(X_SHUTF, HIGH);
        delay(10);
        if (!loxF.init())
        {
            BTSerial.println(F("Loi khoi tao Cam bien F!"));
            while (1)
                ;
        }
        loxF.setAddress(ADDRF);
        BTSerial.print("Cam bien F doi sang 0x");
        BTSerial.println(ADDRF, HEX);
        loxF.setMeasurementTimingBudget(20000);
        loxF.startContinuous();

        BTSerial.println("Khoi dong Cam bien L...");
        digitalWrite(X_SHUTL, HIGH);
        delay(10);
        if (!loxL.init())
        {
            BTSerial.println(F("Loi khoi tao Cam bien L!"));
            while (1)
                ;
        }
        loxL.setAddress(ADDRL);
        BTSerial.print("Cam bien L doi sang 0x");
        BTSerial.println(ADDRL, HEX);
        loxL.setMeasurementTimingBudget(20000);
        loxL.startContinuous();

        BTSerial.println("3 cam bien san sang!");

        defL = loxL.readRangeContinuousMillimeters();
        defR = loxR.readRangeContinuousMillimeters();
        BTSerial.print(defL);
        BTSerial.print(" ");
        BTSerial.println(defR);
    }

    inline float calculate_steering_adjustment()
    {

        float pTerm = 2.0f * m_cross_track_error;
        float dTerm = 4.0f * (m_cross_track_error - m_last_steering_error);
        float adjustment = (pTerm + dTerm) * LOOP_INTERVAL;
        adjustment = constrain(adjustment, -90, 90);
        m_last_steering_error = m_cross_track_error;
        m_steering_adjustment = adjustment;
        return adjustment;
    }

    void set_steering_mode(uint8_t mode)
    {
        m_last_steering_error = m_cross_track_error;
        m_steering_adjustment = 0;
        g_steering_mode = mode;
    }

    void enable()
    {

        m_active = true;
    }

    void disable()
    {

        m_active = false;
    }

    void update(int x)
    {
        if (not m_active)
        {
            m_cross_track_error = 0;
            m_steering_adjustment = 0;
            return;
        }
        uint16_t current_range_left, current_range_front, current_range_right;
        distL = loxL.readRangeContinuousMillimeters();
        distR = loxR.readRangeContinuousMillimeters();
        distF = loxF.readRangeContinuousMillimeters();

        lss.raw = distL;
        rss.raw = distR;
        ff.raw = distF;

        lss.value = ((lss.raw));
        rss.value = ((rss.raw));
        ff.value = ff.raw;

        see_left_wall = lss.value < defL * 1.10f;
        see_right_wall = rss.value < defR * 1.10f;
        see_front_wall = ff.value < 300;
        m_front_sum = ff.value;

        int error = 0;
        float left_error = lss.value * 100 / defL - 100;
        float right_error = rss.value * 100 / defR - 100;

        if (g_steering_mode == STEER_NORMAL)
        {
            if (sensors.see_left_wall && sensors.see_right_wall)
            {
                error = left_error - right_error;
            }
            else if (sensors.see_left_wall)
            {
                error = 2.0 * left_error;
            }
            else if (sensors.see_right_wall)
            {
                error = -2.0 * right_error;
            }
        }
        else if (g_steering_mode == STEER_LEFT_WALL)
        {
            error = 2.0 * left_error;
        }
        else if (g_steering_mode == STEER_RIGHT_WALL)
        {
            error = -2.0 * right_error;
        }

        if (ff.value < 100)
        {
            error = 0;
        }
        m_cross_track_error = error;
    }

    bool occluded_left()
    {
        return lss.value < 70;
    }

    bool occluded_right()
    {
        return rss.value < 70;
    }

    uint8_t wait_for_user_start()
    {
        int state = 0;
        digitalWrite(PC13, 1);
        enable();
        uint8_t choice = NO_START;
        while (choice == NO_START)
        {
            delay(200);
            for (int i = 0; i < 10; i++)
            {
                sensors.update(1);
                delay(20);
            }
            BTSerial.println(lss.value);
            int count = 0;
            if (occluded_left())
            {
                choice = LEFT_START;
                break;
            }
            while (occluded_right())
            {
                choice = RIGHT_START;
                break;
            }
            digitalWrite(PC13, state);
            state = 1 - state;
            delay(25);
        }
        disable();
        digitalWrite(PC13, 0);
        delay(250);
        return choice;
    }

private:
    float m_last_steering_error = 0;
    volatile bool m_active = false;
    volatile float m_cross_track_error = 0;
    volatile float m_steering_adjustment = 0;
    volatile float m_front_sum = 0;
    volatile float m_front_diff = 0;
};

#endif