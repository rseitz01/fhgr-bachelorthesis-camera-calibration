/*
 * cli.c
 *
 *  Created on: Jun 20, 2024
 *      Author: rapha
 */

#include "cli.h"
#include "try.h"
#include "auto_mode.h"
#include "main.h"
#include <inttypes.h>
#include <ctype.h>

int cli_init(Cli *cli, Tlc *tlc, AutoMode *am)
{
	if(!cli) return -1;
	if(!tlc) return -1;
	TRY(str_reserve(&cli->str, 4096));
	cli->tlc = tlc;
	cli->auto_mode = am;
	cli->ready = true;
	cli_process(cli);
	return 0;
error:
	return -1;
}


int cli_feed(Cli *cli, char *str, size_t len)
{
	if(!cli) return -1;
	if(!str) return -1;
	if(!len) return 0;
	for(size_t i = 0; i < len; ++i) {
		char c = str[i];
		if(c == '\n' || c == 'r') {
		}
		if(cli->escape_next) {
			cli->escape_next = false;
			continue;
		}
		switch(c) {
			case 127: { // sometimes is backspace (other systems)
				cli_raw_send("\b ");
				if(str_length(&cli->str)) {
					--cli->str.last;
					cli_raw_send("\b");
				}
			} break;
			case 8: { // backspace
				cli_raw_send(" ");
				if(str_length(&cli->str)) {
					--cli->str.last;
					cli_raw_send("\b");
				}
			} break;
			case 12: { // clear screen (ctrl+l)
				str_clear(&cli->str);
				cli->ready = true;
			} break;
			case 3: { // interrupt (ctrl+c)
				str_clear(&cli->str);
				cli->ready = true;
				cli_raw_send("^C (Interrupt Signal)\r\n");
			} break;
			case '\\':
				//cli->escape_next = true;
			default: {
				str_app_c(&cli->str, c); // TODO:RETVAL
#if 0
				char buf[32] = {0};
				snprintf(buf, 32, "%u == %c\r\n", str[i], str[i]);
				CDC_Transmit_FS(buf, 32);
#endif
			} break;
		}
	}
	return 0;
error:
	return -1;
}


int cli_parse(Cli *cli)
{
	if(!cli) return -1;

#if 0
	char buf[256] = {0};
	snprintf(buf, 256, "\r\n[%u..%u]:%.*s\r\n", cli->str.first, cli->str.last, (int)str_length(&cli->str), str_iter_begin(&cli->str));
	cli_raw_send(buf);
#endif

	if(!str_length(&cli->str)) return 0;
	if(str_rfind_c(&cli->str, '\n') < str_length(&cli->str) ||
			str_rfind_c(&cli->str, '\r') < str_length(&cli->str)) {

		cli_raw_send("\n\r"); // terminal doesn't do \n for us

		cli_parse_help(cli);
		cli_parse_info(cli);
		cli_parse_config(cli, cli->str, false);
		cli_parse_mode(cli);
		cli->ready = true;

	}
#if 0
	if(str_rfind_c(&cli->str, 3) < str_length(&cli->str)) {
		cli_raw_send("^C (Interrupt Signal)\r\n");
		str_clear(&cli->str);
		cli_ready();
	}
#endif

	return 0;
}


int cli_parse_help(Cli *cli)
{
	if(str_cmp_cstr_start(&cli->str, "help")) return 0;
	cli_raw_send("    help\r\n");
	cli_raw_send("    info\r\n");
	cli_raw_send("    config\r\n");
	cli_raw_send("    mode\r\n");
	return 0;
}

int cli_info(Cli *cli, CliInfoList id)
{
	if(!cli) return -1;
	switch(id) {
	case CLI_INFO__COUNT:
	case CLI_INFO_NONE: {} break;
	case CLI_INFO_VERSION: {
		char buf[256] = {0};
		// hardware!
		// software!
		snprintf(buf, 256, "Version: xyz-HW / xyz-SW\r\n");
		cli_raw_send(buf);
	} break;
	case CLI_INFO_CLOCK: {
		char buf[256] = {0};
		snprintf(buf, 256, "Clock Frequency: %" PRIu32 " Hz\r\n", cli->tlc->freq.system);
		cli_raw_send(buf);
	} break;
	case CLI_INFO_ARR: {
		char buf[256] = {0};
		snprintf(buf, 256, "Auto-Reload: %" PRIu32 "\r\n", cli->tlc->rider.arr);
		cli_raw_send(buf);
	} break;
	case CLI_INFO_PSC: {
		char buf[256] = {0};
		snprintf(buf, 256, "Prescaler: %" PRIu32 "\r\n", cli->tlc->rider.psc);
		cli_raw_send(buf);
	} break;
	case CLI_INFO_FPS: {
		char buf[256] = {0};
		snprintf(buf, 256, "FPS: %f\r\n", cli->tlc->freq.fps);
		cli_raw_send(buf);
	} break;
	case CLI_INFO__ALL: {
		for(CliInfoList id = CLI_INFO_NONE + 1; id < CLI_INFO__COUNT; ++id) {
			cli_info(cli, id);
		}
	} break;
	case CLI_INFO_GPIO_OUT: {
		char buf[256] = {0};
		snprintf(buf, 256, "GPIO Output: %s\r\n", cli->tlc->gpio_out ? "On" : "Off");
		cli_raw_send(buf);
	} break;
	case CLI_INFO_GPIO_INT: {
		char buf[256] = {0};
		snprintf(buf, 256, "GPIO Interrupt: %s\r\n", cli->tlc->gpio_int ? "On" : "Off");
		cli_raw_send(buf);
	} break;
	case CLI_INFO__USAGE: {
		cli_raw_send("    all			print all of the following:\r\n"); // keep first, so it makes sense
		cli_raw_send("    version		print version\r\n");
		cli_raw_send("    clock    		print current clock frequency\r\n");
		cli_raw_send("    arr			print current ARR (auto-reload) value\r\n");
		cli_raw_send("    psc			print current PSC (prescaler) value\r\n");
		cli_raw_send("    fps			print FPS (frames per second) derived from ARR & PSC\r\n");
		cli_raw_send("    out			print state of gpio output\r\n");
		cli_raw_send("    int			print state of gpio interrupt\r\n");
	} break;
	}
	return 0;
}

int cli_process(Cli *cli)
{
	if(!cli) return -1;
	cli_info(cli, cli->info);
	cli_config(cli, 0, CLI_CONFIG__APPLY);
	cli_mode(cli, 0, CLI_MODE__APPLY);
	cli->info = CLI_INFO_NONE;

	if(cli->ready) {
		// done
		str_clear(&cli->str);
		cli_ready();
		cli->ready = false;
	}
	if(cli->reset) {
		cli_raw_send("RESETTING in 1000 ms...");
		HAL_Delay(1000);
		NVIC_SystemReset();
	}

	return 0;
}

int cli_parse_info(Cli *cli)
{
	Str arg = STR("info");
	if(str_cmp_start(&cli->str, &arg, true)) return 0;

	Str snip = cli->str;
	snip.first += str_length(&arg);
	snip.first += str_find_nws(&snip);

	if(!str_cmp_start(&snip, &STR("all"), true)) {
		cli->info = CLI_INFO__ALL;
	} else if(!str_cmp_start(&snip, &STR("version"), true)) {
		cli->info = CLI_INFO_VERSION;
	} else if(!str_cmp_start(&snip, &STR("clock"), true)) {
		cli->info = CLI_INFO_CLOCK;
	} else if(!str_cmp_start(&snip, &STR("arr"), true)) {
		cli->info = CLI_INFO_ARR;
	} else if(!str_cmp_start(&snip, &STR("psc"), true)) {
		cli->info = CLI_INFO_PSC;
	} else if(!str_cmp_start(&snip, &STR("fps"), true)) {
		cli->info = CLI_INFO_FPS;
	} else if(!str_cmp_start(&snip, &STR("out"), true)) {
		cli->info = CLI_INFO_GPIO_OUT;
	} else if(!str_cmp_start(&snip, &STR("int"), true)) {
		cli->info = CLI_INFO_GPIO_INT;
	} else {
		cli->info = CLI_INFO__USAGE;
	}

	return 0;
}

int cli_config(Cli *cli, Str *str, CliConfigList id)
{
	char cstr[256] = {0};
	char *endptr = 0;
	switch(id) {
	case CLI_CONFIG__COUNT:
	case CLI_CONFIG_NONE: {} break;
	case CLI_CONFIG__USAGE: {
		cli_raw_send("    arr			set ARR (auto-reload) value\r\n");
		cli_raw_send("    psc			set PSC (prescaler) value\r\n");
		cli_raw_send("    out			set GPIO output (start auto mode -> output)\r\n");
		cli_raw_send("    int			set GPIO input (interrupt -> start auto mode)\r\n");
		cli_raw_send("    fps			set FPS (frames per second); calculates values for ARR & PSC\r\n");
		cli_raw_send("    reset 		trigger software reset\r\n");
	} break;
	case CLI_CONFIG_RESET: {
		if(str->s) {
			cli->reset = true;
			str->s = 0;
		}
	} break;
	case CLI_CONFIG_ARR: {
		if(str_length(str)) {
			str_cstr(str, cstr, 256);
			uint32_t val = strtoul(cstr, &endptr, 0);
			if((!*endptr || isspace((int)*endptr)) && (endptr != cstr)) {
				tlc_set_arr(cli->tlc, val);
				cli_raw_send("Successfully applied ARR.\r\n");
			} else {
				cli_raw_send("Invalid number specified.\r\n");
			}
			str_clear(str);
		}
	} break;
	case CLI_CONFIG_PSC: {
		if(str_length(str)) {
			str_cstr(str, cstr, 256);
			uint32_t val = strtoul(cstr, &endptr, 0);
			if((!*endptr || isspace((int)*endptr)) && (endptr != cstr)) {
				tlc_set_psc(cli->tlc, val);
				cli_raw_send("Successfully applied PSC.\r\n");
			} else {
				cli_raw_send("Invalid number specified.\r\n");
			}
			str_clear(str);
		}
	} break;
	case CLI_CONFIG_FPS: {
		if(str_length(str)) {
			cli_raw_send("Estimating best FPS configuration...\r\n");
			str_cstr(str, cstr, 256);
			float fps = strtof(cstr, &endptr);
			if((!*endptr || isspace((int)*endptr)) && (endptr != cstr)) {
				tlc_set_fps(cli->tlc, fps);
				cli_raw_send("Successfully applied FPS (PSC & ARR).\r\n");
			} else {
				cli_raw_send("Invalid number specified.\r\n");
			}
			str_clear(str);
		}
	} break;
	case CLI_CONFIG_GPIO_OUT: {
		if(str_length(str)) {
			if(!str_cmp_start(str, &STR("on"), true)) {
				cli->tlc->gpio_out = true;
				cli_raw_send("Turned on GPIO output.\r\n");
			} else if(!str_cmp_start(str, &STR("off"), true)) {
				cli->tlc->gpio_out = false;
				cli_raw_send("Turned off GPIO output.\r\n");
			} else {
				cli_raw_send("    out=on\r\n");
				cli_raw_send("    out=off\r\n");
			}
			str_clear(str);
		}
	} break;
	case CLI_CONFIG_GPIO_INT: {
		if(str_length(str)) {
			if(!str_cmp_start(str, &STR("on"), true)) {
				cli->tlc->gpio_int = true;
				cli_raw_send("Turned on GPIO interrupt.\r\n");
			} else if(!str_cmp_start(str, &STR("off"), true)) {
				cli->tlc->gpio_int = false;
				cli_raw_send("Turned off GPIO interrupt.\r\n");
			} else {
				cli_raw_send("    int=on\r\n");
				cli_raw_send("    int=off\r\n");
			}
			str_clear(str);
		}
	} break;
	case CLI_CONFIG__APPLY: {
		for(CliConfigList id = CLI_CONFIG_NONE + 1; id < CLI_CONFIG__COUNT; ++id) {
			cli_config(cli, &cli->config_snip[id], id);
		}
	} break;
	}
	return 0;
}

int cli_parse_config(Cli *cli, Str str, bool skip_check)
{
	if(!cli) return -1;

	Str snip = {0};

	if(!skip_check) {
		Str arg = STR("config");
		if(str_cmp_start(&str, &arg, true)) return 0;

		snip = cli->str;
		snip.first += str_length(&arg);
		snip.first += str_find_nws(&snip);
	}

	Str args[CLI_CONFIG__COUNT] = {
			[CLI_CONFIG_ARR] = STR("arr="),
			[CLI_CONFIG_PSC] = STR("psc="),
			[CLI_CONFIG_FPS] = STR("fps="),
			[CLI_CONFIG_GPIO_OUT] = STR("out="),
			[CLI_CONFIG_GPIO_INT] = STR("int="),
			[CLI_CONFIG_RESET] = STR("reset"),
	};
	//Str arg = {0};
	for(size_t id = CLI_CONFIG_NONE + 1; id < CLI_CONFIG__COUNT; ++id) {
		Str arg = args[id];
		if(str_cmp_start(&snip, &arg, false)) continue;
		/* get data snippet */
		switch(id) { // TODO: I don't think I need the switch? also in cli_parse_mode ...
		case CLI_CONFIG_ARR:
		case CLI_CONFIG_PSC:
		case CLI_CONFIG_GPIO_OUT:
		case CLI_CONFIG_GPIO_INT:
		case CLI_CONFIG_FPS: {
			size_t end = str_find_ws(&snip);
			Str data = snip;
			data.first += str_length(&arg);
			data.last = data.first + end;
			cli->config_snip[id] = data;
		} break;
		default: {
			cli->config_snip[id] = snip; // TODO make void *
		} break;
		}
		return 0; // TODO make sure we can just do: config arr=xy psc=ab (two or more ones at the same time)
		/* next argument (see TODO above // not done yet */
		Str snip = str;
		snip.first += str_length(&arg);
		snip.first += str_find_nws(&snip);
		str = snip;

	}
	cli_config(cli, 0, CLI_CONFIG__USAGE);

#if 0
	if(str_length(&arg)) {
		snip.first += str_length(&arg);
		snip.first += str_find_nws(&snip);
		str = snip;
	}
#endif


	return 0;
}

void cli_mode(Cli *cli, Str *str, CliModeList id)
{
	if(!cli) return;
	char cstr[256] = {0};
	char *endptr = 0;
	bool deactivate_auto = false;
	switch(id) {
	case CLI_MODE__COUNT: {} break;
	case CLI_MODE__USAGE: {
		cli_raw_send("    none			off\r\n");
		cli_raw_send("    auto			automatic mode\r\n");
		cli_raw_send("    pwm=VAL 		only do PWM on all LEDs\r\n");
		cli_raw_send("    rider 		let a single LED be lit at all times. matches FPS (ARR & PSC)\r\n");
		cli_raw_send("    display=VAL	show pattern\r\n");
	} break;
	case CLI_MODE_NONE: {
		if(str->s) { // TODO make this better (void *)
			tlc_mode_exit(cli->tlc, id);
			tlc_run(cli->tlc, false);
			deactivate_auto = true;
			str->s = 0; // TODO this is so horrendously stupid. but it works.
		}
	} break;
	case CLI_MODE_AUTO: {
		if(str->s) {
			auto_mode_activate(cli->auto_mode, cli->tlc->freq.fps);
			if(cli->tlc->gpio_out) {
				HAL_GPIO_WritePin(GPIO_Out_GPIO_Port, GPIO_Out_Pin, GPIO_PIN_SET);
			}
			str->s = 0; // TODO this is so horrendously stupid. but it works.
		}
	} break;
	case CLI_MODE_PWM: {
		if(str_length(str)) {
			tlc_mode_exit(cli->tlc, id);
			str_cstr(str, cstr, 256);
			uint32_t val = strtoul(cstr, &endptr, 0);
			if((!*endptr || isspace((int)*endptr)) && endptr != cstr && val >= 0 && val <= TLC_PWM_MAX) {
				*cli->tlc->pwm.duty_cycle = val;
				cli_raw_send("Successfully applied PWM.\r\n");
			} else {
				char buf[256] = {0};
				snprintf(buf, 256, "Invalid number specified. (Range = 0..%u, where 0=0%% and %u=100%%)\r\n", TLC_PWM_MAX, TLC_PWM_MAX);
				cli_raw_send(buf);
			}
			tlc_mode(cli->tlc, TLC_MODE_PWM);
			tlc_run(cli->tlc, true);
			deactivate_auto = true;
			str_clear(str);
		}
	} break;
	case CLI_MODE_RIDER: {
		if(str->s) { // TODO make this better (void *)
			tlc_mode(cli->tlc, TLC_MODE_RIDER);
			tlc_run(cli->tlc, true);
			deactivate_auto = true;
			str->s = 0; // TODO this is so horrendously stupid. but it works.
		}
	} break;
	case CLI_MODE_DISPLAY: {
		if(str_length(str)) {
			tlc_mode_exit(cli->tlc, id);
			if(str_length(str) < TLC_N_LEDS) {
				cli_raw_send("Not enough bits specified (16 required)\r\n"); // TODO str_fmt
				str_clear(str);
				return;
			}
			for(size_t i = 0; i < TLC_N_LEDS; ++i) {
				char ci = str_at(str, i);
				if(ci == '1') {
					cli->tlc->display.data[i] = 1;
				} else if(ci == '0') {
					cli->tlc->display.data[i] = 0;
				} // TODO handle if ci != 0 and ci != 1 !! -> what do? x = keep? or what ?!?!
			}
			tlc_mode(cli->tlc, TLC_MODE_DISPLAY);
			tlc_run(cli->tlc, true);
			deactivate_auto = true;
			str_clear(str);
		}
	} break;
	case CLI_MODE__APPLY: {
		for(CliModeList id = CLI_MODE_NONE; id < CLI_MODE__COUNT; ++id) {
			cli_mode(cli, &cli->mode[id], id);
		}
	} break;
	}
	if(deactivate_auto) {
		auto_mode_deactivate(cli->auto_mode);
		if(cli->tlc->gpio_out) {
			HAL_GPIO_WritePin(GPIO_Out_GPIO_Port, GPIO_Out_Pin, GPIO_PIN_RESET);
		}
	}
}

int cli_parse_mode(Cli *cli)
{
	Str arg = STR("mode");
	if(str_cmp_start(&cli->str, &arg, true)) return 0;

	Str snip = cli->str;
	snip.first += str_length(&arg);
	snip.first += str_find_nws(&snip);

	Str args[CLI_MODE__COUNT] = {
			[CLI_MODE_AUTO] = STR("auto"),
			[CLI_MODE_NONE] = STR("none"),
			[CLI_MODE_PWM] = STR("pwm="),
			[CLI_MODE_RIDER] = STR("rider"),
			[CLI_MODE_DISPLAY] = STR("display="),
	};

	for(size_t id = CLI_MODE_NONE; id < CLI_MODE__COUNT; ++id) {
		Str arg = args[id];
		if(str_cmp_start(&snip, &arg, false)) continue;
		/* get data snippet */
		switch(id) {
		case CLI_MODE_PWM:
		case CLI_MODE_DISPLAY: {
			size_t end = str_find_ws(&snip);
			Str data = snip;
			data.first += str_length(&arg);
			data.last = data.first + end;
			cli->mode[id] = data;
		} break;
		default: {
			cli->mode[id] = snip; // TODO make void *
		} break;
		}
		return 0;
	}
	cli_mode(cli, 0, CLI_MODE__USAGE);

	return 0;
}


void cli_raw_send(char *str)
{
	size_t len = strlen(str);
	while(CDC_Transmit_FS((uint8_t *)str, len) == USBD_BUSY) {};
}

void cli_ready(void)
{
	cli_raw_send("$ ");
}
