/*
 * cli.h
 *
 *  Created on: Jun 20, 2024
 *      Author: rapha
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_

#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

#include "str.h"
#include "tlc.h"
#include "auto_mode.h"

typedef enum {
	CLI_INFO_NONE,
	/* list below */
	CLI_INFO_VERSION,
	CLI_INFO_CLOCK,
	CLI_INFO_PSC,
	CLI_INFO_ARR,
	CLI_INFO_FPS,
	CLI_INFO_GPIO_OUT,
	CLI_INFO_GPIO_INT,
	/* list above */
	CLI_INFO__COUNT, // keep exactly here
	CLI_INFO__ALL,
	CLI_INFO__USAGE,
} CliInfoList;

typedef enum {
	CLI_CONFIG_NONE,
	/* list below */
	CLI_CONFIG_ARR,
	CLI_CONFIG_PSC,
	CLI_CONFIG_FPS,
	CLI_CONFIG_GPIO_OUT,
	CLI_CONFIG_GPIO_INT,
	CLI_CONFIG_RESET,
	/* list above */
	CLI_CONFIG__COUNT, // keep exactly here
	CLI_CONFIG__USAGE,
	CLI_CONFIG__APPLY,
} CliConfigList;

typedef enum {
	/* list below */
	CLI_MODE_NONE,
	CLI_MODE_AUTO,
	CLI_MODE_PWM,
	CLI_MODE_RIDER,
	CLI_MODE_DISPLAY,
	/* list above */
	CLI_MODE__COUNT, // keep exactly here
	CLI_MODE__USAGE,
	CLI_MODE__APPLY,
} CliModeList;

typedef struct Cli {
	Str str;
	Tlc *tlc;
	AutoMode *auto_mode;
	CliInfoList info;
	CliConfigList config;
	Str config_snip[CLI_CONFIG__COUNT];
	Str mode[CLI_MODE__COUNT];
	bool ready;
	bool reset;
	bool escape_next;
} Cli;

int cli_init(Cli *cli, Tlc *tlc, AutoMode *am);
void cli_ready(void);
int cli_feed(Cli *cli, char *str, size_t len);
int cli_parse(Cli *cli);


int cli_info(Cli *cli, CliInfoList id);
int cli_config(Cli *cli, Str *str, CliConfigList id);

int cli_parse_help(Cli *cli);
int cli_parse_info(Cli *cli);
int cli_parse_config(Cli *cli, Str str, bool skip_check);
int cli_parse_mode(Cli *cli);

void cli_mode(Cli *cli, Str *str, CliModeList id);
int cli_process(Cli *cli);

void cli_raw_send(char *str);


#endif /* INC_CLI_H_ */
