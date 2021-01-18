#pragma once

#include <string>

enum Colors {
    FG_DEFAULT = 39,
    BG_DEFAULT = 49,
    FG_BLACK = 30,
    BG_BLACK = 40,
    FG_RED = 31,
    BG_RED = 41,
    FG_GREEN = 32,
    BG_GREEN = 42,
    FG_YELLOW = 33,
    BG_YELLOW = 43,
    FG_BLUE = 34,
    BG_BLUE = 44,
    FG_MAGENTA = 35,
    BG_MAGENTA = 45,
    FG_CYAN = 36,
    BG_CYAN = 46,
    FG_WHITE = 37,
    BG_WHITE = 47,
    FG_BLACK_BRIGHT = 90,
    BG_BLACK_BRIGHT = 100,
    FG_RED_BRIGHT = 91,
    BG_RED_BRIGHT = 101,
    FG_GREEN_BRIGHT = 92,
    BG_GREEN_BRIGHT = 102,
    FG_YELLOW_BRIGHT = 93,
    BG_YELLOW_BRIGHT = 103,
    FG_BLUE_BRIGHT = 94,
    BG_BLUE_BRIGHT = 104,
    FG_MAGENTA_BRIGHT = 95,
    BG_MAGENTA_BRIGHT = 105,
    FG_CYAN_BRIGHT = 96,
    BG_CYAN_BRIGHT = 106,
    FG_WHITE_BRIGHT = 97,
    BG_WHITE_BRIGHT = 107,
    BOLD = 1,
    FAINT = 2,
    UNDERLINE = 4,
    DOUBLE_UNDERLINE = 21,
    BLINK = 5,
};



static std::string colorize(const std::string &msg, const Colors &color) {
    std::string colorful_msg = "\033[1;";
    colorful_msg.append(std::to_string(static_cast<int>(color)));
    colorful_msg.append("m");
    colorful_msg.append(msg);
    colorful_msg.append("\033[0m");
    return colorful_msg;
}