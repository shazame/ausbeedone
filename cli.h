#ifndef CLI_H
#define CLI_H

#include "control_system.h"

#define CLI_END_CHAR  '\r'
#define CLI_DELIMITER ' '

void cli_start(struct control_system *);

#endif /* CLI_H */
