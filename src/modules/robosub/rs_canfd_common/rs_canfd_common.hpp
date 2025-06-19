#pragma once

union converter{
    uint8_t bytes[4];
    float value;
};

// Enum of available HU Robosub module IDs
typedef enum
{
  GLOBAL    = 0x00,
  PIXHAWK   = 0x10,
  MAINBRAIN = 0x11,
  BUOYANCY  = 0x04,
  HYDRAULIC = 0X05,
  POWER     = 0x08,
  TEST      = 0x0F
} modules;
