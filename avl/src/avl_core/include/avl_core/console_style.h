//==============================================================================
// Autonomous Vehicle Library
//
// Description: A selection of ANSI/VT100 Control sequence definitions.
//              See: https://misc.flogisoft.com/bash/tip_colors_and_formatting
//              NOTE: We're using a flag of 255 to indicate "no style".  This
//              is not a VT100 standard but rather our policy/choice.
//==============================================================================

#ifndef CONSOLE_STYLE_H
#define CONSOLE_STYLE_H

namespace CONSOLE_STYLE
{

    static const unsigned char style_reset     = 0;
    static const unsigned char style_bold      = 1;
    static const unsigned char style_dim       = 2;
    static const unsigned char style_italic    = 3;
    static const unsigned char style_underline = 4;
    static const unsigned char style_blink     = 5;
    static const unsigned char style_normal    = 255;

    static const unsigned char fg_default      = 39;
    static const unsigned char fg_black        = 30;
    static const unsigned char fg_red          = 91;
    static const unsigned char fg_green        = 92;
    static const unsigned char fg_yellow       = 93;
    static const unsigned char fg_blue         = 94;
    static const unsigned char fg_magenta      = 35;
    static const unsigned char fg_cyan         = 36;
    static const unsigned char fg_gray         = 37;
    static const unsigned char fg_white        = 97;
    static const unsigned char fg_orange       = 208;

    static const unsigned char bg_default      = 49;
    static const unsigned char bg_black        = 40;
    static const unsigned char bg_red          = 41;
    static const unsigned char bg_green        = 42;
    static const unsigned char bg_yellow       = 43;
    static const unsigned char bg_blue         = 44;
    static const unsigned char bg_magenta      = 45;
    static const unsigned char bg_cyan         = 46;
    static const unsigned char bg_gray         = 47;

}

#endif // CONSOLE_STYLE_H
