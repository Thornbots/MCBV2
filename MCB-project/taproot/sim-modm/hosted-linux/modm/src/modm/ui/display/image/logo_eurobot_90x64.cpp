/*
 * Copyright (c) 2011, Fabian Greif
 * Copyright (c) 2013, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/architecture/interface/accessor.hpp>

namespace bitmap
{
FLASH_STORAGE(uint8_t logo_eurobot_90x64[]) = {
    90,   64,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xc0, 0xc0, 0xc0, 0xe0, 0xe0, 0xe0,
    0xf0, 0xf0, 0xf0, 0xf0, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xfc, 0xbc, 0x1c, 0xbc, 0xfc,
    0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xf8, 0xfa, 0xfa, 0xfa, 0x7a, 0x3a, 0x7a, 0xf4, 0xf4,
    0xf4, 0xec, 0xec, 0xe8, 0xf8, 0xd8, 0xd0, 0xb0, 0xb0, 0x60, 0x60, 0xc0, 0xc0, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0xf0, 0x78, 0x38, 0xbc, 0xbe, 0x3e,
    0x7f, 0xff, 0x3f, 0x3f, 0xff, 0xff, 0x3f, 0x3f, 0xff, 0x3d, 0x38, 0x7d, 0x3f, 0x3f, 0xff, 0x7f,
    0x3f, 0x3f, 0x3f, 0x3f, 0x7f, 0xff, 0x07, 0x07, 0x3f, 0x3f, 0x3f, 0x7f, 0xff, 0xff, 0x7f, 0x3f,
    0x3f, 0x3f, 0x3f, 0x7f, 0xff, 0x3f, 0x0e, 0x0f, 0x3f, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xef, 0xcf,
    0xe7, 0xcf, 0xef, 0xff, 0xff, 0xfe, 0xfe, 0xfd, 0xfb, 0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xe0, 0xc0,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xf0, 0xfc, 0xfe,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xe0, 0xed, 0xed, 0xec, 0xec, 0xff, 0xf0, 0xe0, 0xef, 0xe7,
    0xf0, 0xe0, 0xff, 0xe0, 0xe0, 0xfe, 0xff, 0xff, 0xf0, 0xe0, 0xef, 0xef, 0xef, 0xe0, 0xf0, 0xff,
    0xe0, 0xe0, 0xef, 0xef, 0xe0, 0xf0, 0xff, 0xf0, 0xe0, 0xef, 0xef, 0xef, 0xe0, 0xf0, 0xff, 0xff,
    0xe0, 0xe0, 0xef, 0xef, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdb, 0xeb,
    0xe3, 0xf1, 0xe3, 0xcb, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xf8, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0x3f, 0x3f, 0x3f, 0x1f, 0x1f, 0x1f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f,
    0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x03, 0x39, 0xf1, 0xe1, 0x01,
    0x01, 0x01, 0x03, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0x7f, 0x3f, 0x7f, 0xff, 0xff, 0xfb, 0xdb, 0xe3, 0xf0, 0xf1, 0xf3, 0xe3, 0xdb, 0xfb, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff,
    0xfc, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xf7, 0xf3, 0xe3, 0xe3, 0xe3, 0xc3, 0xc3, 0xc3,
    0x81, 0x81, 0x81, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x30, 0x78, 0xfc, 0xfe, 0xf9, 0xf3, 0xf1, 0xf0, 0x78, 0x38, 0x38, 0x3e, 0x3f, 0x3f,
    0x3f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xf6, 0xf2, 0xf8, 0xf8, 0xfc, 0xf8, 0xf8, 0xf2,
    0xf6, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xbf, 0x5f, 0x2f, 0x1f, 0x07, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1f, 0x1f, 0x1f, 0x0c, 0x0c, 0x0c, 0x1c,
    0x3e, 0x3e, 0x7f, 0xff, 0xff, 0x7f, 0x7f, 0x3f, 0x3f, 0x3f, 0x7f, 0xff, 0xff, 0x07, 0x00, 0x00,
    0x00, 0x00, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xf8, 0xf8, 0xf8, 0xf0, 0xf0, 0xf0, 0xe0, 0xe0, 0xc1,
    0xc3, 0xc7, 0x80, 0x08, 0x1c, 0x9c, 0x9c, 0xf8, 0xf8, 0xe0, 0x00, 0x00, 0x00, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x7f, 0x7f, 0xbf, 0xbf, 0xdf, 0x5f, 0x6f, 0x2f, 0x17, 0x13,
    0x0b, 0x05, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0xf8, 0xe0, 0x80, 0x70, 0xf0,
    0xe0, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x18, 0x1e, 0x1f, 0x1f, 0x3e, 0x3e, 0x3d,
    0x3d, 0x3d, 0x3d, 0x3d, 0x3d, 0x7b, 0x7b, 0x7b, 0x7b, 0x7b, 0x7b, 0x3b, 0x3b, 0x3b, 0x3a, 0x3a,
    0x38, 0x39, 0x39, 0x38, 0x18, 0x1a, 0x1d, 0x1d, 0x0d, 0x0d, 0x0d, 0x0e, 0x06, 0x06, 0x06, 0x03,
    0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0f, 0x1e, 0x3d, 0x3b, 0x77, 0x67, 0x66, 0x60, 0x20,
    0x10, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00,
};
}
