#ifndef CLI_H
#define CLI_H

#include "trajectory_manager.h"

#define CLI_BUFFER_SIZE  80
#define CLI_MAX_ARGS     8

#define CLI_END_CHAR  '\r'
#define CLI_DEL_CHAR  127

// When an arrow key is pressed, an ANSI escape sequence is generated
// It is composed of 3 chars: 27, 91 and a char to identify the arrow key
#define CLI_ESC_CHAR  27
#define CLI_UP_KEY    65
#define CLI_DOWN_KEY  66
#define CLI_RIGHT_KEY 67
#define CLI_LEFT_KEY  68

#define CLI_DELIMITER " "

void cli_start(struct trajectory_manager *);

#endif /* CLI_H */
