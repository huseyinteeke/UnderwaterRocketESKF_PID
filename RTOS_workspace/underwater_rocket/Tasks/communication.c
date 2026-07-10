/*
 * communication.c
 *
 *  Created on: Jan 23, 2026
 *      Author: husey
 */


#include "tasks_config.h"
#include <string.h>
#include <stdlib.h>


extern char msg_buffer[50];
extern UART_HandleTypeDef huart3;


uint8_t Comm_Get_Parsed_Command(ParsedCommand_t* output_cmd)
{
    output_cmd->type = CMD_NONE;

    char temp_buffer[50];
    strcpy(temp_buffer, msg_buffer);
    char *token = strtok(temp_buffer, ":");

    if(token == NULL) return false;

    // --- Command seperation ---
    if(strcmp(token, "ARM") == 0) {
        output_cmd->type = CMD_ARM;
    }
    else if(strcmp(token, "DISARM") == 0) {
        output_cmd->type = CMD_DISARM;
    }
    else if(strcmp(token, "PIDP") == 0) {
        output_cmd->type = CMD_SET_PID_PITCH;

        char* p = strtok(NULL, ":");
        char* i = strtok(NULL, ":");
        char* d = strtok(NULL, ":");

        if(p && i && d) {
            output_cmd->val1 = atof(p);
            output_cmd->val2 = atof(i);
            output_cmd->val3 = atof(d);
        }
    }
    else if(strcmp(token, "PIDY") == 0) {
            output_cmd->type = CMD_SET_PID_YAW;

            char* p = strtok(NULL, ":");
            char* i = strtok(NULL, ":");
            char* d = strtok(NULL, ":");

            if(p && i && d) {
                output_cmd->val1 = atof(p);
                output_cmd->val2 = atof(i);
                output_cmd->val3 = atof(d);
            }
    }
    else if(strcmp(token , "LED") == 0){
      output_cmd->type = CMD_LED;
    }

    return (output_cmd->type != CMD_NONE);
}


void Comm_Send_Response(const char* msg)
{
    SEGGER_SYSVIEW_Print(msg);
}

