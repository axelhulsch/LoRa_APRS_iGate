/*
    created with FontEditor written by H. Reddmann
    HaReddmann at t-online dot de

    File Name           : HoloLens_12.h
    Date                : 10.03.2019
    Font size in bytes  : 0x0D64, 3428
    Font width          : 13
    Font height         : 17
    Font first char     : 0x0B
    Font last char      : 0xFF
    Font bits per pixel : 1
    Font is compressed  : false

    The font data are defined as

    struct _FONT_ {
     // common shared fields
       uint16_t   font_Size_in_Bytes_over_all_included_Size_it_self;
       uint8_t    font_Width_in_Pixel_for_fixed_drawing;
       uint8_t    font_Height_in_Pixel_for_all_Characters;
       uint8_t    font_Bits_per_Pixels;
                    // if MSB are set then font is a compressed font
       uint8_t    font_First_Char;
       uint8_t    font_Last_Char;
       uint8_t    font_Char_Widths[font_Last_Char - font_First_Char +1];
                    // for each character the separate width in pixels,
                    // characters < 128 have an implicit virtual right empty row
                    // characters with font_Char_Widths[] == 0 are undefined

     // if compressed font then additional fields
       uint8_t    font_Byte_Padding;
                    // each Char in the table are aligned in size to this value
       uint8_t    font_RLE_Table[3];
                    // Run Length Encoding Table for compression
       uint8_t    font_Char_Size_in_Bytes[font_Last_Char - font_First_Char +1];
                    // for each char the size in (bytes / font_Byte_Padding) are stored,
                    // this get us the table to seek to the right beginning of each char
                    // in the font_data[].

     // for compressed and uncompressed fonts
       uint8_t    font_data[];
                    // bit field of all characters
    }
*/

#include "FontDesc.h"

#ifndef HoloLens_12_FONT_H
#define HoloLens_12_FONT_H

#define HoloLens_12_WIDTH 13
#define HoloLens_12_HEIGHT 17

static unsigned char const HoloLens_12_Bytes[] = {
    0x04, 0x0A, 0x00, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x04, 0x08, 0x06, 0x0A, 0x07, 0x02, 0x04, 0x04, 0x06, 
    0x06, 0x03, 0x05, 0x02, 0x04, 0x06, 0x04, 0x06, 0x06, 0x07, 0x06, 0x06, 0x06, 0x06, 0x06, 0x02, 
    0x03, 0x06, 0x06, 0x06, 0x06, 0x0B, 0x08, 0x07, 0x08, 0x08, 0x07, 0x07, 0x08, 0x08, 0x02, 0x06, 
    0x08, 0x07, 0x0A, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x08, 0x08, 0x08, 0x0C, 0x09, 0x08, 0x06, 
    0x03, 0x04, 0x03, 0x06, 0x08, 0x03, 0x06, 0x06, 0x06, 0x06, 0x06, 0x04, 0x06, 0x06, 0x02, 0x03, 
    0x06, 0x02, 0x0A, 0x06, 0x06, 0x06, 0x06, 0x04, 0x05, 0x04, 0x06, 0x06, 0x0A, 0x07, 0x06, 0x05, 
    0x04, 0x02, 0x04, 0x07, 0x04, 0x07, 0x00, 0x04, 0x07, 0x06, 0x09, 0x06, 0x06, 0x04, 0x10, 0x08, 
    0x04, 0x0C, 0x00, 0x07, 0x00, 0x00, 0x04, 0x04, 0x06, 0x06, 0x05, 0x07, 0x0D, 0x06, 0x0A, 0x06, 
    0x04, 0x0B, 0x00, 0x06, 0x08, 0x00, 0x03, 0x07, 0x07, 0x07, 0x07, 0x03, 0x07, 0x04, 0x0A, 0x05, 
    0x07, 0x07, 0x05, 0x0A, 0x07, 0x05, 0x07, 0x05, 0x05, 0x04, 0x08, 0x07, 0x03, 0x04, 0x04, 0x05, 
    0x07, 0x0A, 0x0B, 0x0A, 0x07, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0C, 0x09, 0x08, 0x08, 0x08, 
    0x08, 0x03, 0x04, 0x04, 0x04, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x07, 0x09, 0x09, 0x09, 
    0x09, 0x09, 0x08, 0x09, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x0B, 0x07, 0x07, 0x07, 0x07, 
    0x07, 0x03, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 
    0x07, 0x07, 0x06, 0x07, 0x06, 
    0xE0, 0x7F, 0xC0, 0xFF, 0x80, 0x00, 0x01, 0x01, 0x02, 0xC0, 0x01, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 
    0x3F, 0x80, 0x7F, 0x00, 0xFF, 0x00, 0x3C, 0x00, 0xF0, 0x00, 0xC0, 0x03, 0x00, 0x07, 0xF8, 0x1F, 
    0xF0, 0x3F, 0x20, 0x40, 0x40, 0x80, 0x80, 0xFF, 0x01, 0xFF, 0x03, 0x02, 0x04, 0x04, 0x08, 0xF8, 
    0x1F, 0xF0, 0x3F, 0x20, 0x40, 0x40, 0x80, 0x80, 0xFF, 0x01, 0xFF, 0x03, 0x02, 0x04, 0x04, 0x08, 
    0xF8, 0x1F, 0xF0, 0x3F, 0x20, 0x40, 0x40, 0x80, 0x80, 0xFF, 0x01, 0xFF, 0x03, 0x02, 0x04, 0x04, 
    0x08, 0xF8, 0x1F, 0xF0, 0x3F, 0x20, 0x40, 0x40, 0x80, 0x80, 0xFF, 0x01, 0xFF, 0x03, 0x02, 0x04, 
    0x04, 0x08, 0xF8, 0x1F, 0xF0, 0x3F, 0x20, 0x40, 0x40, 0x80, 0x80, 0xFF, 0x01, 0xFF, 0x03, 0x02, 
    0x04, 0x04, 0x08, 0xF8, 0x1F, 0xF0, 0x3F, 0x20, 0x40, 0x40, 0x80, 0x80, 0xFF, 0x01, 0xFF, 0x03, 
    0x02, 0x04, 0x04, 0x08, 0xF8, 0x1F, 0xF0, 0x3F, 0x20, 0x40, 0x40, 0x80, 0x80, 0xFF, 0x01, 0xFF, 
    0x03, 0x02, 0x04, 0x04, 0x08, 0xF0, 0x17, 0xE0, 0x2F, 0xC0, 0x01, 0x80, 0x03, 0x00, 0x07, 0x00, 
    0x0E, 0x00, 0x00, 0x01, 0x20, 0x0E, 0xC0, 0x1F, 0xE0, 0x3F, 0xC0, 0x7F, 0x80, 0x3F, 0x00, 0x47, 
    0x00, 0x08, 0x00, 0x38, 0x02, 0xF8, 0x0C, 0xF8, 0x3F, 0xF0, 0x7F, 0xC0, 0x7C, 0x00, 0x71, 0x00, 
    0x06, 0x00, 0x1E, 0x01, 0x24, 0x03, 0x78, 0x03, 0x60, 0x03, 0x00, 0x1B, 0x00, 0x7B, 0x00, 0x93, 
    0x00, 0xE2, 0x01, 0x80, 0x01, 0x80, 0x03, 0xB0, 0x0F, 0xF0, 0x11, 0x20, 0x27, 0xC0, 0x7B, 0x00, 
    0xF3, 0x00, 0x60, 0x01, 0x0E, 0x00, 0x1C, 0x00, 0xC0, 0x07, 0xE0, 0x3F, 0xE0, 0xE0, 0x40, 0x00, 
    0x81, 0x00, 0x02, 0x07, 0x07, 0xFC, 0x07, 0xE0, 0x03, 0x10, 0x00, 0xA0, 0x00, 0xE0, 0x01, 0xC0, 
    0x03, 0x00, 0x05, 0x00, 0x02, 0x00, 0x20, 0x00, 0x40, 0x00, 0xE0, 0x03, 0xC0, 0x07, 0x00, 0x02, 
    0x00, 0x04, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x02, 0x80, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 
    0x04, 0x00, 0x08, 0x00, 0x80, 0x00, 0x00, 0x01, 0x00, 0x03, 0xF0, 0x07, 0xF8, 0x03, 0x30, 0x00, 
    0xC0, 0x1F, 0xC0, 0x7F, 0x80, 0x80, 0x00, 0x01, 0x01, 0xFE, 0x03, 0xF8, 0x03, 0x10, 0x00, 0x20, 
    0x00, 0xE0, 0x3F, 0xC0, 0x7F, 0x00, 0xC1, 0x00, 0xC3, 0x01, 0xC2, 0x02, 0xC4, 0x04, 0xF8, 0x08, 
    0xE0, 0x10, 0x40, 0x10, 0xC0, 0x60, 0x80, 0x88, 0x00, 0x11, 0x01, 0xFE, 0x03, 0xB8, 0x03, 0x00, 
    0x03, 0x00, 0x07, 0x80, 0x0B, 0x80, 0x13, 0x80, 0xFF, 0x00, 0xFF, 0x01, 0x80, 0x00, 0x7C, 0x02, 
    0xF8, 0x0C, 0x90, 0x10, 0x20, 0x21, 0x40, 0x7E, 0x80, 0x78, 0x00, 0xFE, 0x00, 0xFE, 0x03, 0x64, 
    0x04, 0x48, 0x08, 0xB0, 0x1F, 0x40, 0x1E, 0x40, 0x00, 0x80, 0xE0, 0x00, 0xF1, 0x01, 0x7A, 0x00, 
    0x3C, 0x00, 0x18, 0x00, 0xE0, 0x0E, 0xE0, 0x3F, 0x40, 0x44, 0x80, 0x88, 0x00, 0xFF, 0x01, 0xDC, 
    0x01, 0x78, 0x02, 0xF8, 0x0D, 0x10, 0x12, 0x20, 0x26, 0xC0, 0x7F, 0x00, 0x7F, 0x00, 0x08, 0x01, 
    0x10, 0x02, 0x00, 0x08, 0x40, 0x18, 0x80, 0x10, 0x00, 0x02, 0x00, 0x0E, 0x00, 0x36, 0x00, 0x44, 
    0x00, 0x8C, 0x01, 0x08, 0x02, 0x40, 0x01, 0x80, 0x02, 0x00, 0x05, 0x00, 0x0A, 0x00, 0x14, 0x00, 
    0x28, 0x00, 0x04, 0x01, 0x18, 0x03, 0x20, 0x02, 0xC0, 0x06, 0x00, 0x07, 0x00, 0x04, 0x00, 0x01, 
    0x00, 0x03, 0x00, 0xC2, 0x02, 0xC4, 0x05, 0xF8, 0x00, 0xE0, 0x00, 0x00, 0x0F, 0x80, 0x7F, 0x00, 
    0xC3, 0x00, 0x33, 0x03, 0xF2, 0x04, 0x24, 0x09, 0xC8, 0x13, 0xB0, 0x27, 0xC0, 0x48, 0x80, 0x1F, 
    0x00, 0x3C, 0x00, 0x80, 0x01, 0xE0, 0x03, 0xF0, 0x01, 0x78, 0x02, 0xF0, 0x04, 0x80, 0x0F, 0x00, 
    0x7C, 0x00, 0xC0, 0x00, 0xFF, 0x01, 0xFE, 0x03, 0x44, 0x04, 0x88, 0x08, 0x10, 0x11, 0xE0, 0x3F, 
    0x80, 0x3B, 0x00, 0x3E, 0x00, 0xFE, 0x00, 0x06, 0x03, 0x04, 0x04, 0x08, 0x08, 0x30, 0x18, 0xC0, 
    0x18, 0x00, 0x11, 0x80, 0xFF, 0x00, 0xFF, 0x01, 0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x30, 0x18, 
    0xC0, 0x1F, 0x00, 0x1F, 0x80, 0xFF, 0x00, 0xFF, 0x01, 0x22, 0x02, 0x44, 0x04, 0x88, 0x08, 0x10, 
    0x11, 0x20, 0x20, 0xC0, 0x7F, 0x80, 0xFF, 0x00, 0x11, 0x00, 0x22, 0x00, 0x44, 0x00, 0x88, 0x00, 
    0x10, 0x00, 0x80, 0x0F, 0x80, 0x3F, 0x80, 0xC1, 0x00, 0x01, 0x01, 0x22, 0x02, 0x4C, 0x06, 0xB0, 
    0x0F, 0x40, 0x1F, 0xE0, 0x3F, 0xC0, 0x7F, 0x00, 0x08, 0x00, 0x10, 0x00, 0x20, 0x00, 0x40, 0x00, 
    0xF8, 0x0F, 0xF0, 0x1F, 0xE0, 0x3F, 0xC0, 0x7F, 0x00, 0x60, 0x00, 0xC0, 0x01, 0x00, 0x02, 0x00, 
    0x04, 0xF8, 0x0F, 0xF0, 0x0F, 0xE0, 0x3F, 0xC0, 0x7F, 0x00, 0x0C, 0x00, 0x3C, 0x00, 0xCC, 0x00, 
    0x0C, 0x03, 0x08, 0x0C, 0x00, 0x10, 0xE0, 0x3F, 0xC0, 0x7F, 0x00, 0x80, 0x00, 0x00, 0x01, 0x00, 
    0x02, 0x00, 0x04, 0x00, 0x08, 0xF0, 0x1F, 0xE0, 0x3F, 0x00, 0x07, 0x00, 0x3C, 0x00, 0xE0, 0x00, 
    0xC0, 0x01, 0xE0, 0x01, 0xE0, 0x00, 0xF0, 0x1F, 0xE0, 0x3F, 0xC0, 0x7F, 0x80, 0xFF, 0x00, 0x0E, 
    0x00, 0x38, 0x00, 0xC0, 0x01, 0x00, 0x07, 0xF0, 0x1F, 0xE0, 0x3F, 0x00, 0x1F, 0x00, 0x7F, 0x00, 
    0x83, 0x01, 0x02, 0x02, 0x04, 0x04, 0x18, 0x0C, 0xE0, 0x0F, 0x80, 0x0F, 0xC0, 0x7F, 0x80, 0xFF, 
    0x00, 0x11, 0x00, 0x22, 0x00, 0x44, 0x00, 0x88, 0x00, 0xF0, 0x01, 0xC0, 0x01, 0x00, 0x1F, 0x00, 
    0x7F, 0x00, 0x83, 0x01, 0x02, 0x02, 0x04, 0x05, 0x18, 0x0E, 0xE0, 0x1F, 0x80, 0x2F, 0xC0, 0x7F, 
    0x80, 0xFF, 0x00, 0x11, 0x00, 0x22, 0x00, 0x44, 0x00, 0x88, 0x01, 0xF0, 0x1F, 0xC0, 0x39, 0x80, 
    0x33, 0x80, 0xEF, 0x00, 0x11, 0x01, 0x22, 0x02, 0x44, 0x04, 0xB8, 0x0F, 0x60, 0x0E, 0x20, 0x00, 
    0x40, 0x00, 0x80, 0x00, 0x00, 0xFF, 0x01, 0xFE, 0x03, 0x04, 0x00, 0x08, 0x00, 0x10, 0x00, 0xE0, 
    0x0F, 0xC0, 0x3F, 0x00, 0xC0, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x06, 0xF8, 0x07, 0xF0, 0x07, 
    0x60, 0x00, 0xC0, 0x07, 0x00, 0x3E, 0x00, 0xE0, 0x01, 0xC0, 0x03, 0xF0, 0x01, 0xF8, 0x00, 0x30, 
    0x00, 0x60, 0x00, 0xC0, 0x07, 0x00, 0x3E, 0x00, 0xE0, 0x01, 0xC0, 0x03, 0xF0, 0x01, 0xE0, 0x03, 
    0x00, 0x1E, 0x00, 0x3C, 0x00, 0x1F, 0x80, 0x0F, 0x00, 0x03, 0x00, 0x02, 0x02, 0x0C, 0x06, 0x30, 
    0x06, 0xC0, 0x07, 0x00, 0x07, 0x00, 0x1F, 0x00, 0x63, 0x00, 0x83, 0x01, 0x02, 0x02, 0x0C, 0x00, 
    0x38, 0x00, 0xC0, 0x00, 0x00, 0x3F, 0x00, 0x7E, 0x00, 0x06, 0x00, 0x07, 0x00, 0x06, 0x00, 0x04, 
    0x06, 0x08, 0x0F, 0x10, 0x17, 0xA0, 0x23, 0xC0, 0x43, 0x80, 0x81, 0x00, 0xFF, 0x07, 0xFE, 0x0F, 
    0x04, 0x10, 0x18, 0x00, 0xF0, 0x07, 0x80, 0x3F, 0x00, 0x60, 0x80, 0x00, 0x02, 0xFF, 0x07, 0xFE, 
    0x0F, 0x10, 0x00, 0x30, 0x00, 0x30, 0x00, 0x60, 0x00, 0x80, 0x01, 0x00, 0x02, 0x00, 0x00, 0x04, 
    0x00, 0x08, 0x00, 0x10, 0x00, 0x20, 0x00, 0x40, 0x00, 0x80, 0x00, 0x00, 0x01, 0x00, 0x02, 0x01, 
    0x00, 0x06, 0x00, 0x08, 0x00, 0x00, 0x06, 0x80, 0x1E, 0x00, 0x25, 0x00, 0x4A, 0x00, 0xFC, 0x00, 
    0xF0, 0x01, 0xFE, 0x03, 0xFC, 0x07, 0x40, 0x08, 0x80, 0x10, 0x00, 0x3F, 0x00, 0x3C, 0x00, 0x78, 
    0x00, 0xF8, 0x01, 0x10, 0x02, 0x20, 0x04, 0xC0, 0x0C, 0x00, 0x09, 0x00, 0x1E, 0x00, 0x7E, 0x00, 
    0x84, 0x00, 0x08, 0x01, 0xFE, 0x03, 0xFC, 0x07, 0x80, 0x07, 0x80, 0x1F, 0x00, 0x25, 0x00, 0x4A, 
    0x00, 0xDC, 0x00, 0xB0, 0x00, 0x10, 0x00, 0xF8, 0x07, 0xF8, 0x0F, 0x90, 0x00, 0x00, 0x9E, 0x00, 
    0x7E, 0x01, 0x84, 0x02, 0x08, 0x05, 0xF0, 0x0F, 0xE0, 0x0F, 0xF8, 0x0F, 0xF0, 0x1F, 0x00, 0x03, 
    0x00, 0x02, 0x00, 0xFC, 0x00, 0xF0, 0x01, 0xF2, 0x03, 0xE4, 0x07, 0x00, 0x20, 0x90, 0x7F, 0x20, 
    0x7F, 0xC0, 0x7F, 0x80, 0xFF, 0x00, 0x70, 0x00, 0xB0, 0x01, 0x20, 0x06, 0x00, 0x08, 0xF0, 0x1F, 
    0xE0, 0x3F, 0x00, 0x7E, 0x00, 0xFC, 0x00, 0x18, 0x00, 0x10, 0x00, 0xE0, 0x07, 0x80, 0x0F, 0x80, 
    0x01, 0x00, 0x01, 0x00, 0x7E, 0x00, 0xF8, 0x00, 0xF8, 0x01, 0xF0, 0x03, 0x60, 0x00, 0x40, 0x00, 
    0x80, 0x1F, 0x00, 0x3E, 0x00, 0x3C, 0x00, 0xFC, 0x00, 0x08, 0x01, 0x10, 0x02, 0xE0, 0x07, 0x80, 
    0x07, 0x80, 0x7F, 0x00, 0xFF, 0x00, 0x42, 0x00, 0x84, 0x00, 0xF8, 0x01, 0xE0, 0x01, 0xC0, 0x03, 
    0xC0, 0x0F, 0x80, 0x10, 0x00, 0x21, 0x00, 0xFE, 0x01, 0xFC, 0x03, 0xF8, 0x01, 0xF0, 0x03, 0x60, 
    0x00, 0x40, 0x00, 0x00, 0x09, 0x00, 0x37, 0x00, 0x5A, 0x00, 0xEC, 0x00, 0x90, 0x00, 0x10, 0x00, 
    0xF8, 0x03, 0xF0, 0x0F, 0x80, 0x10, 0x00, 0x1F, 0x00, 0x7E, 0x00, 0x80, 0x00, 0x80, 0x01, 0xF0, 
    0x03, 0xE0, 0x07, 0xC0, 0x00, 0x80, 0x07, 0x00, 0x3C, 0x00, 0x78, 0x00, 0x3C, 0x00, 0x18, 0x00, 
    0x30, 0x00, 0xE0, 0x03, 0x00, 0x0F, 0x00, 0x1E, 0x00, 0x1F, 0x00, 0x3E, 0x00, 0xF0, 0x00, 0xE0, 
    0x01, 0xF0, 0x01, 0x60, 0x00, 0x40, 0x08, 0x80, 0x19, 0x00, 0x1E, 0x00, 0x18, 0x00, 0x78, 0x00, 
    0x98, 0x01, 0x10, 0x02, 0x60, 0x10, 0xC0, 0x33, 0x00, 0x3E, 0x00, 0x3C, 0x00, 0x1E, 0x00, 0x0C, 
    0x00, 0x88, 0x01, 0x90, 0x03, 0xA0, 0x05, 0xC0, 0x09, 0x80, 0x11, 0x00, 0x04, 0x80, 0xFF, 0x80, 
    0xEF, 0x03, 0x01, 0x04, 0xFE, 0x0F, 0xFC, 0x1F, 0x08, 0x20, 0xF0, 0x7D, 0xC0, 0x7F, 0x00, 0x08, 
    0x00, 0x01, 0x00, 0x03, 0x00, 0x02, 0x00, 0x0C, 0x00, 0x10, 0x00, 0x30, 0x00, 0x20, 0x00, 0xC0, 
    0x7F, 0x80, 0xFF, 0x00, 0xFF, 0x01, 0xFE, 0x03, 0xA0, 0x00, 0xF0, 0x07, 0xF0, 0x1F, 0x20, 0x25, 
    0x40, 0x42, 0x80, 0xE3, 0x00, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x38, 0x00, 0x30, 0x00, 
    0x80, 0x00, 0x00, 0x01, 0xF4, 0x03, 0xFE, 0x03, 0x3E, 0x00, 0x24, 0x00, 0x08, 0x00, 0x00, 0x00, 
    0x00, 0x80, 0x00, 0xC0, 0x01, 0x80, 0x03, 0x00, 0x07, 0x00, 0x06, 0x00, 0x00, 0x00, 0x0C, 0x00, 
    0x18, 0x00, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x06, 0x40, 0x00, 
    0x80, 0x00, 0xE0, 0x1F, 0xC0, 0x3F, 0x00, 0x04, 0x00, 0x08, 0x00, 0x48, 0x00, 0x90, 0x00, 0xF8, 
    0x07, 0xF0, 0x0F, 0x80, 0x04, 0x00, 0x09, 0x00, 0x01, 0x00, 0x03, 0x00, 0x06, 0x00, 0x08, 0x00, 
    0x30, 0x00, 0xF0, 0x10, 0x20, 0x31, 0xC0, 0x3B, 0x00, 0x3B, 0x00, 0x1C, 0x00, 0x9C, 0x01, 0x8C, 
    0x07, 0x08, 0x09, 0x00, 0x1E, 0x00, 0x18, 0x00, 0x30, 0x00, 0xF0, 0x00, 0x20, 0x01, 0xC0, 0x03, 
    0x00, 0x03, 0x00, 0x00, 0xE0, 0x0C, 0xE0, 0x3B, 0x50, 0x44, 0xE0, 0x88, 0xC0, 0x11, 0x81, 0xEE, 
    0x03, 0x98, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x1E, 0x00, 0x24, 0x00, 0x00, 0x00, 0x7C, 0x00, 
    0xFC, 0x01, 0x0C, 0x06, 0x08, 0x08, 0x10, 0x10, 0x60, 0x30, 0xC0, 0x7F, 0x80, 0xFF, 0x00, 0x11, 
    0x01, 0x22, 0x02, 0x04, 0x04, 0x00, 0x00, 0x10, 0x18, 0x28, 0x3C, 0x70, 0x5C, 0xE0, 0x8E, 0x40, 
    0x0F, 0x01, 0x06, 0x02, 0x00, 0x00, 0x20, 0x00, 0x70, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x03, 
    0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x20, 0x00, 0x70, 0x00, 0xE0, 0x00, 0xC0, 0x01, 0x80, 
    0x01, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x1C, 0x00, 0x38, 0x00, 0x70, 0x00, 0x20, 0x00, 0x00, 0x00, 
    0x00, 0x38, 0x00, 0x70, 0x00, 0xE0, 0x00, 0xC0, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x04, 0x00, 
    0x08, 0x00, 0x10, 0x00, 0x20, 0x00, 0x40, 0x00, 0x80, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x04, 
    0x00, 0x08, 0x00, 0x10, 0x00, 0x20, 0x00, 0x40, 0x00, 0x80, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 
    0x04, 0x00, 0x08, 0x40, 0x00, 0xC0, 0x00, 0x80, 0x01, 0x00, 0x03, 0x00, 0x06, 0x00, 0x04, 0x00, 
    0x00, 0x00, 0x40, 0x00, 0x80, 0x07, 0x00, 0x0F, 0x00, 0x1E, 0x00, 0x3C, 0x00, 0x70, 0x00, 0xE0, 
    0x00, 0xE0, 0x01, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x90, 0x00, 0x72, 0x03, 0xAC, 0x05, 0xD8, 0x0E, 
    0x10, 0x09, 0x00, 0x00, 0x00, 0x24, 0x00, 0x78, 0x00, 0x60, 0x00, 0x00, 0x00, 0xC0, 0x03, 0xC0, 
    0x0F, 0x80, 0x10, 0x00, 0x21, 0x00, 0x7E, 0x00, 0xFC, 0x00, 0x28, 0x01, 0x50, 0x02, 0xE0, 0x06, 
    0x80, 0x05, 0x00, 0x00, 0x00, 0x31, 0x40, 0x72, 0x80, 0xB5, 0x00, 0x3B, 0x01, 0x32, 0x02, 0x0C, 
    0x00, 0x38, 0x00, 0xC8, 0x00, 0x10, 0x3F, 0x20, 0x7E, 0x40, 0x06, 0x00, 0x07, 0x00, 0x06, 0x00, 
    0x00, 0x00, 0x40, 0x3F, 0x80, 0x7E, 0x00, 0x00, 0x00, 0x3C, 0x00, 0xFC, 0x01, 0xF8, 0x03, 0xF8, 
    0x03, 0x70, 0x06, 0x80, 0x04, 0x00, 0x00, 0xC0, 0x32, 0xC0, 0x7F, 0x80, 0xBC, 0x00, 0x13, 0x01, 
    0x24, 0x02, 0x00, 0x04, 0x00, 0x00, 0xA0, 0x0B, 0xC0, 0x1F, 0x00, 0x11, 0x00, 0x22, 0x00, 0xFE, 
    0x00, 0x74, 0x01, 0x00, 0x00, 0xB8, 0x02, 0xF0, 0x05, 0x00, 0x3F, 0x00, 0x7E, 0x00, 0x2F, 0x00, 
    0x56, 0x00, 0x00, 0x00, 0x7C, 0x1F, 0xF8, 0x3E, 0x00, 0x00, 0xC0, 0x46, 0xC0, 0x9F, 0x81, 0x64, 
    0x02, 0x99, 0x04, 0xE6, 0x0F, 0x88, 0x0D, 0x10, 0x00, 0x20, 0x00, 0x40, 0x00, 0x80, 0x00, 0x00, 
    0x3E, 0x00, 0xFE, 0x00, 0x76, 0x03, 0xF4, 0x05, 0x28, 0x0A, 0x50, 0x14, 0xA0, 0x28, 0xC0, 0x60, 
    0x00, 0x7F, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x20, 0x01, 0xE8, 0x02, 0xF0, 0x05, 0xC0, 0x0B, 0x00, 
    0x00, 0x00, 0x20, 0x00, 0xE0, 0x00, 0xE0, 0x03, 0xC0, 0x07, 0x80, 0x0D, 0x00, 0x11, 0x00, 0x00, 
    0x00, 0x04, 0x00, 0x08, 0x00, 0x10, 0x00, 0x20, 0x00, 0xC0, 0x00, 0x80, 0x01, 0x00, 0x02, 0x00, 
    0x04, 0x00, 0x08, 0x00, 0x10, 0x00, 0x20, 0x00, 0xF8, 0x00, 0xF8, 0x03, 0x18, 0x0C, 0xD0, 0x17, 
    0xA0, 0x2F, 0x40, 0x4D, 0x80, 0xBE, 0x00, 0xCB, 0x01, 0xFC, 0x01, 0xF0, 0x01, 0x02, 0x00, 0x04, 
    0x00, 0x08, 0x00, 0x10, 0x00, 0x20, 0x00, 0x40, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 
    0x70, 0x00, 0xE0, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x10, 0x01, 0x20, 0x02, 0xF0, 0x05, 0xE0, 
    0x0B, 0x00, 0x11, 0x00, 0x22, 0x80, 0x04, 0x80, 0x0D, 0x00, 0x1D, 0x00, 0x2E, 0x00, 0x48, 0x00, 
    0x50, 0x00, 0xB0, 0x01, 0xA0, 0x02, 0xC0, 0x07, 0x00, 0x05, 0x00, 0x00, 0x00, 0x04, 0x00, 0x0C, 
    0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFE, 0x01, 0x80, 0x00, 0x00, 0x01, 0xF0, 0x03, 
    0xE0, 0x07, 0x00, 0x08, 0xE0, 0x00, 0xE0, 0x03, 0xC0, 0xFF, 0x81, 0xFF, 0x03, 0xFF, 0x07, 0xFE, 
    0x0F, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x06, 
    0x00, 0x0C, 0x00, 0x00, 0x10, 0x00, 0xF0, 0x01, 0xE0, 0x03, 0x00, 0x00, 0x00, 0x27, 0x00, 0x5F, 
    0x00, 0xBE, 0x00, 0x38, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x36, 0x00, 0x7C, 0x00, 0xF8, 0x00, 
    0xE0, 0x00, 0x80, 0x00, 0x08, 0x00, 0xF8, 0x00, 0xF0, 0x11, 0x00, 0x38, 0x00, 0x3C, 0x00, 0x1C, 
    0x00, 0xCE, 0x00, 0xCE, 0x01, 0xC4, 0x07, 0x80, 0x0F, 0x20, 0x00, 0xE0, 0x03, 0xC0, 0x47, 0x00, 
    0xE0, 0x00, 0xF0, 0x00, 0x70, 0x00, 0xB8, 0x04, 0xB8, 0x0D, 0x10, 0x1D, 0x00, 0x2E, 0x00, 0x48, 
    0x00, 0x05, 0x00, 0x1B, 0x00, 0x2A, 0x02, 0x7C, 0x07, 0xD0, 0x07, 0x80, 0x03, 0xC0, 0x19, 0xC0, 
    0x39, 0x80, 0xF8, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x3C, 0x80, 0x4E, 0x00, 0x8D, 
    0x00, 0x80, 0x01, 0x00, 0x01, 0x80, 0x01, 0xE0, 0x03, 0xF1, 0x01, 0x7E, 0x02, 0xF8, 0x04, 0x80, 
    0x0F, 0x00, 0x7C, 0x00, 0xC0, 0x00, 0x80, 0x01, 0xE0, 0x03, 0xF0, 0x01, 0x7C, 0x02, 0xFC, 0x04, 
    0x88, 0x0F, 0x00, 0x7C, 0x00, 0xC0, 0x00, 0x80, 0x01, 0xE0, 0x03, 0xF2, 0x01, 0x7E, 0x02, 0xFC, 
    0x04, 0x90, 0x0F, 0x00, 0x7C, 0x00, 0xC0, 0x00, 0x80, 0x01, 0xE1, 0x03, 0xF3, 0x01, 0x7E, 0x02, 
    0xFC, 0x04, 0x98, 0x0F, 0x10, 0x7C, 0x00, 0xC0, 0x00, 0x80, 0x01, 0xE1, 0x03, 0xF2, 0x01, 0x78, 
    0x02, 0xF0, 0x04, 0x90, 0x0F, 0x20, 0x7C, 0x00, 0xC0, 0x00, 0x80, 0x01, 0xE0, 0x03, 0xF1, 0x01, 
    0x7F, 0x02, 0xFE, 0x04, 0x88, 0x0F, 0x00, 0x7C, 0x00, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x03, 0x80, 
    0x03, 0xC0, 0x01, 0xE0, 0x03, 0xE0, 0x04, 0xC0, 0x7F, 0x80, 0xFF, 0x00, 0x11, 0x01, 0x22, 0x02, 
    0x44, 0x04, 0x08, 0x08, 0x00, 0x00, 0x80, 0x0F, 0x80, 0x3F, 0x80, 0xC1, 0x00, 0x01, 0x05, 0x02, 
    0x0E, 0x0C, 0x1E, 0x30, 0x06, 0x40, 0x04, 0x00, 0x00, 0xC0, 0x7F, 0x80, 0xFF, 0x40, 0x11, 0x81, 
    0x23, 0x02, 0x46, 0x04, 0x88, 0x08, 0x10, 0x10, 0x00, 0x00, 0xC0, 0x7F, 0x80, 0xFF, 0x00, 0x11, 
    0x01, 0x23, 0x02, 0x47, 0x04, 0x8A, 0x08, 0x10, 0x10, 0x00, 0x00, 0xC0, 0x7F, 0x80, 0xFF, 0x80, 
    0x11, 0x81, 0x23, 0x02, 0x47, 0x04, 0x8C, 0x08, 0x10, 0x10, 0x00, 0x00, 0xC0, 0x7F, 0xA0, 0xFF, 
    0x40, 0x11, 0x01, 0x22, 0x02, 0x45, 0x04, 0x8A, 0x08, 0x10, 0x10, 0x08, 0x00, 0xF0, 0x7F, 0xC0, 
    0xFF, 0x00, 0x00, 0x00, 0xFF, 0x03, 0xFF, 0x07, 0x02, 0x00, 0x08, 0x00, 0xF8, 0x3F, 0xF0, 0x7F, 
    0x40, 0x00, 0x40, 0x00, 0x80, 0xFE, 0x03, 0xFD, 0x07, 0x02, 0x00, 0x00, 0x01, 0xE0, 0x3F, 0xC0, 
    0x7F, 0x80, 0x88, 0x00, 0x11, 0x01, 0x02, 0x02, 0x0C, 0x06, 0xF0, 0x07, 0xC0, 0x07, 0x00, 0x00, 
    0xC0, 0x7F, 0xC0, 0xFF, 0xC0, 0x0E, 0x80, 0x39, 0x00, 0xC3, 0x01, 0x06, 0x07, 0xF4, 0x1F, 0xE0, 
    0x3F, 0x00, 0x00, 0x00, 0x3E, 0x00, 0xFE, 0x80, 0x06, 0x03, 0x07, 0x04, 0x0C, 0x08, 0x30, 0x18, 
    0xC0, 0x1F, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x7C, 0x00, 0xFC, 0x01, 0x0C, 0x06, 0x0C, 0x08, 0x1C, 
    0x10, 0x68, 0x30, 0x80, 0x3F, 0x00, 0x3E, 0x00, 0x00, 0x00, 0xF8, 0x00, 0xF8, 0x03, 0x1C, 0x0C, 
    0x1C, 0x10, 0x38, 0x20, 0xE0, 0x60, 0x00, 0x7F, 0x00, 0x7C, 0x00, 0x00, 0x00, 0xF0, 0x01, 0xF4, 
    0x07, 0x3C, 0x18, 0x38, 0x20, 0x70, 0x40, 0xE0, 0xC1, 0x40, 0xFE, 0x00, 0xF8, 0x00, 0x00, 0x00, 
    0xE0, 0x03, 0xE0, 0x0F, 0x68, 0x30, 0x50, 0x40, 0xA0, 0x80, 0x40, 0x83, 0x01, 0xFC, 0x01, 0xF0, 
    0x01, 0x00, 0x00, 0x40, 0x04, 0x80, 0x0D, 0x00, 0x0E, 0x00, 0x1C, 0x00, 0x6C, 0x00, 0x88, 0x00, 
    0x00, 0x00, 0xE0, 0x0B, 0xE0, 0x1F, 0x60, 0x3C, 0x40, 0x5C, 0x80, 0x8E, 0x00, 0x8F, 0x01, 0xFE, 
    0x01, 0xF4, 0x01, 0x00, 0x00, 0xF0, 0x07, 0xE0, 0x1F, 0x10, 0x60, 0x60, 0x80, 0x80, 0x00, 0x01, 
    0x00, 0x03, 0xFC, 0x03, 0xF8, 0x03, 0x00, 0x00, 0xE0, 0x0F, 0xC0, 0x3F, 0x00, 0xC0, 0x80, 0x00, 
    0x81, 0x01, 0x02, 0x01, 0x06, 0xF8, 0x07, 0xF0, 0x07, 0x00, 0x00, 0xC0, 0x1F, 0x80, 0x7F, 0x80, 
    0x80, 0x81, 0x01, 0x02, 0x03, 0x04, 0x04, 0x0C, 0xF0, 0x0F, 0xE0, 0x0F, 0x00, 0x00, 0x80, 0x3F, 
    0x00, 0xFF, 0x80, 0x00, 0x03, 0x01, 0x04, 0x02, 0x08, 0x04, 0x18, 0xE0, 0x1F, 0xC0, 0x1F, 0x80, 
    0x01, 0x00, 0x07, 0x00, 0x18, 0x00, 0xE2, 0x07, 0xC6, 0x0F, 0xC4, 0x00, 0xE0, 0x00, 0xC0, 0x00, 
    0x00, 0x00, 0x00, 0xFF, 0x01, 0xFE, 0x03, 0x10, 0x01, 0x20, 0x02, 0x40, 0x04, 0x80, 0x08, 0x00, 
    0x1F, 0x00, 0x1C, 0x00, 0x00, 0x00, 0xFC, 0x03, 0xFC, 0x07, 0xC8, 0x08, 0xF0, 0x13, 0xC0, 0x3C, 
    0x00, 0x30, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xD2, 0x03, 0xAC, 0x04, 0x50, 0x09, 0x80, 0x1F, 0x00, 
    0x3E, 0x00, 0x00, 0x00, 0x60, 0x00, 0xE8, 0x01, 0x54, 0x02, 0xAC, 0x04, 0xC8, 0x0F, 0x00, 0x1F, 
    0x00, 0x00, 0x00, 0x30, 0x00, 0xF5, 0x00, 0x2B, 0x01, 0x56, 0x02, 0xE8, 0x07, 0x80, 0x0F, 0x00, 
    0x00, 0x40, 0x18, 0xC0, 0x7A, 0x80, 0x95, 0x00, 0x2B, 0x01, 0xF6, 0x03, 0xC4, 0x07, 0x00, 0x00, 
    0x00, 0x0C, 0x40, 0x3D, 0x80, 0x4A, 0x00, 0x95, 0x00, 0xFA, 0x01, 0xE0, 0x03, 0x00, 0x00, 0x00, 
    0x06, 0x90, 0x1E, 0x70, 0x25, 0xE0, 0x4A, 0x80, 0xFC, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x03, 
    0x40, 0x0F, 0x80, 0x12, 0x00, 0x25, 0x00, 0x7E, 0x00, 0xFC, 0x00, 0x28, 0x01, 0x50, 0x02, 0xE0, 
    0x06, 0x80, 0x05, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x7E, 0x01, 0x84, 0x03, 0x08, 0x07, 0x30, 0x03, 
    0x40, 0x02, 0x00, 0x00, 0x00, 0x0F, 0x20, 0x3F, 0xC0, 0x4A, 0x00, 0x95, 0x00, 0xB8, 0x01, 0x60, 
    0x01, 0x00, 0x00, 0x80, 0x07, 0x80, 0x1F, 0x40, 0x25, 0xC0, 0x4A, 0x80, 0xDC, 0x00, 0xB0, 0x00, 
    0x00, 0x00, 0xC0, 0x03, 0xD0, 0x0F, 0xB0, 0x12, 0x60, 0x25, 0x80, 0x6E, 0x00, 0x58, 0x00, 0x00, 
    0x00, 0xE0, 0x01, 0xE8, 0x07, 0x50, 0x09, 0xA0, 0x12, 0x40, 0x37, 0x00, 0x2C, 0x80, 0x00, 0x00, 
    0xFB, 0x01, 0xF4, 0x03, 0x00, 0x00, 0xD0, 0x0F, 0xB0, 0x1F, 0x20, 0x00, 0x80, 0x00, 0x80, 0xFD, 
    0x00, 0xFB, 0x01, 0x04, 0x00, 0x08, 0x00, 0xD0, 0x0F, 0xA0, 0x1F, 0x40, 0x00, 0x00, 0x00, 0x00, 
    0x70, 0x00, 0xF5, 0x01, 0x2E, 0x02, 0x7C, 0x04, 0xE8, 0x0F, 0x00, 0x0F, 0x00, 0x00, 0x80, 0x7E, 
    0x80, 0xFD, 0x00, 0x1B, 0x00, 0x16, 0x00, 0xEC, 0x07, 0x88, 0x0F, 0x00, 0x00, 0x00, 0x1E, 0x40, 
    0x7E, 0x80, 0x85, 0x00, 0x0A, 0x01, 0xF0, 0x03, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x3F, 
    0x80, 0x42, 0x80, 0x85, 0x00, 0xF9, 0x01, 0xE0, 0x01, 0x00, 0x00, 0x80, 0x07, 0xA0, 0x1F, 0x60, 
    0x21, 0xC0, 0x42, 0x00, 0xFD, 0x00, 0xF0, 0x00, 0x00, 0x00, 0xC8, 0x03, 0xD8, 0x0F, 0xB0, 0x10, 
    0x60, 0x21, 0xC0, 0x7E, 0x80, 0x78, 0x00, 0x00, 0x00, 0xE0, 0x01, 0xE8, 0x07, 0x50, 0x08, 0xA0, 
    0x10, 0x40, 0x3F, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x10, 0x00, 0x20, 0x00, 0x50, 0x01, 0xA0, 0x02, 
    0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0xF8, 0x00, 0xF8, 0x01, 0xD0, 0x03, 0xE0, 0x05, 0xC0, 
    0x0F, 0x80, 0x0F, 0x00, 0x00, 0x00, 0x3E, 0x80, 0xFC, 0x00, 0x03, 0x01, 0x04, 0x03, 0xE0, 0x07, 
    0xC0, 0x0F, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x7E, 0x00, 0x81, 0x00, 0x83, 0x01, 0xF2, 0x03, 0xE0, 
    0x07, 0x00, 0x00, 0x80, 0x0F, 0x40, 0x3F, 0xC0, 0x40, 0x80, 0xC1, 0x00, 0xFA, 0x01, 0xF0, 0x03, 
    0x00, 0x00, 0xC0, 0x07, 0xA0, 0x1F, 0x40, 0x20, 0x80, 0x60, 0x00, 0xFD, 0x00, 0xF8, 0x01, 0x30, 
    0x08, 0xE0, 0x19, 0x10, 0x1F, 0x30, 0x1E, 0x20, 0x0F, 0x00, 0x06, 0x00, 0x00, 0x00, 0xFF, 0x07, 
    0xFE, 0x0F, 0x20, 0x04, 0x40, 0x08, 0x80, 0x1F, 0x00, 0x1E, 0x00, 0x06, 0x01, 0x3D, 0x03, 0xE2, 
    0x03, 0xC4, 0x03, 0xE8, 0x01, 0xC0, 0x00, 0x00
};

static struct fontDesc_t const HoloLens_12_Desc = {
   sizeof(HoloLens_12_Bytes),         // total Size
   13,                       // width in pixel
   17,                       // height in pixel
   1,                       // bits per pixel
   0x0B,                  // Code of first char
   0xFF,                  // Code of last char
   HoloLens_12_Bytes                  // Data
};

#endif

