#ifndef CLI_H
#define CLI_H

#include "trajectory_manager.h"

#define CLI_BUFFER_SIZE  80
#define CLI_MAX_ARGS     8

#define CLI_END_CHAR  '\r'
#define CLI_DEL_CHAR  127
#define CLI_DELIMITER " "

void cli_start(struct trajectory_manager *);

#endif /* CLI_H */
