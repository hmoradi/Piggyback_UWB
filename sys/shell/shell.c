/*
 * Copyright (C) 2009, Freie Universitaet Berlin (FUB).
 * Copyright (C) 2013, INRIA.
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell
 * @{
 *
 * @file
 * @brief       Implementation of a very simple command interpreter.
 *              For each command (i.e. "echo"), a handler can be specified.
 *              If the first word of a user-entered command line matches the
 *              name of a handler, the handler will be called with the whole
 *              command line as parameter.
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      René Kijewski <rene.kijewski@fu-berlin.de>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "shell.h"
#include "shell_commands.h"

#if !defined(SHELL_NO_ECHO) || !defined(SHELL_NO_PROMPT)
#ifdef MODULE_NEWLIB
/* use local copy of putchar, as it seems to be inlined,
 * enlarging code by 50% */
static void _putchar(int c) {
    putchar(c);
}
#else
#define _putchar putchar
#endif
#endif

static shell_command_handler_t find_handler(const shell_command_t *command_list, char *command)
{
    const shell_command_t *command_lists[] = {
        command_list,
#ifdef MODULE_SHELL_COMMANDS
        _shell_command_list,
#endif
    };

    const shell_command_t *entry;

    /* iterating over command_lists */
    for (unsigned int i = 0; i < sizeof(command_lists) / sizeof(entry); i++) {
        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                if (strcmp(entry->name, command) == 0) {
                    return entry->handler;
                }
                else {
                    entry++;
                }
            }
        }
    }

    return NULL;
}

static void print_help(const shell_command_t *command_list)
{
    printf("%-20s %s\n", "Command", "Description");
    puts("---------------------------------------");

    const shell_command_t *command_lists[] = {
        command_list,
#ifdef MODULE_SHELL_COMMANDS
        _shell_command_list,
#endif
    };

    const shell_command_t *entry;

    /* iterating over command_lists */
    for (unsigned int i = 0; i < sizeof(command_lists) / sizeof(entry); i++) {
        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                printf("%-20s %s\n", entry->name, entry->desc);
                entry++;
            }
        }
    }
}

static void handle_input_line(const shell_command_t *command_list, char *line)
{
    static const char *INCORRECT_QUOTING = "shell: incorrect quoting";

    /* first we need to calculate the number of arguments */
    unsigned argc = 0;
    char *pos = line;
    int contains_esc_seq = 0;
    while (1) {
        if ((unsigned char) *pos > ' ') {
            /* found an argument */
            if (*pos == '"' || *pos == '\'') {
                /* it's a quoted argument */
                const char quote_char = *pos;
                do {
                    ++pos;
                    if (!*pos) {
                        puts(INCORRECT_QUOTING);
                        return;
                    }
                    else if (*pos == '\\') {
                        /* skip over the next character */
                        ++contains_esc_seq;
                        ++pos;
                        if (!*pos) {
                            puts(INCORRECT_QUOTING);
                            return;
                        }
                        continue;
                    }
                } while (*pos != quote_char);
                if ((unsigned char) pos[1] > ' ') {
                    puts(INCORRECT_QUOTING);
                    return;
                }
            }
            else {
                /* it's an unquoted argument */
                do {
                    if (*pos == '\\') {
                        /* skip over the next character */
                        ++contains_esc_seq;
                        ++pos;
                        if (!*pos) {
                            puts(INCORRECT_QUOTING);
                            return;
                        }
                    }
                    ++pos;
                    if (*pos == '"') {
                        puts(INCORRECT_QUOTING);
                        return;
                    }
                } while ((unsigned char) *pos > ' ');
            }

            /* count the number of arguments we got */
            ++argc;
        }

        /* zero out the current position (space or quotation mark) and advance */
        if (*pos > 0) {
            *pos = 0;
            ++pos;
        }
        else {
            break;
        }
    }
    if (!argc) {
        return;
    }

    /* then we fill the argv array */
    char *argv[argc + 1];
    argv[argc] = NULL;
    pos = line;
    for (unsigned i = 0; i < argc; ++i) {
        while (!*pos) {
            ++pos;
        }
        if (*pos == '"' || *pos == '\'') {
            ++pos;
        }
        argv[i] = pos;
        while (*pos) {
            ++pos;
        }
    }
    for (char **arg = argv; contains_esc_seq && *arg; ++arg) {
        for (char *c = *arg; *c; ++c) {
            if (*c != '\\') {
                continue;
            }
            for (char *d = c; *d; ++d) {
                *d = d[1];
            }
            if (--contains_esc_seq == 0) {
                break;
            }
        }
    }

    /* then we call the appropriate handler */
    shell_command_handler_t handler = find_handler(command_list, argv[0]);
    if (handler != NULL) {
        handler(argc, argv);
    }
    else {
        if (strcmp("help", argv[0]) == 0) {
            print_help(command_list);
        }
        else {
            printf("shell: command not found: %s\n", argv[0]);
        }
    }
}

static int readline(char *buf, size_t size)
{
    char *line_buf_ptr = buf;

    while (1) {
        if ((line_buf_ptr - buf) >= ((int) size) - 1) {
            return -1;
        }

        int c = getchar();
        if (c < 0) {
            return 1;
        }

        /* We allow Unix linebreaks (\n), DOS linebreaks (\r\n), and Mac linebreaks (\r). */
        /* QEMU transmits only a single '\r' == 13 on hitting enter ("-serial stdio"). */
        /* DOS newlines are handled like hitting enter twice, but empty lines are ignored. */
        if (c == '\r' || c == '\n') {
            //puts("shell end of line \r\n");
            *line_buf_ptr = '\0';
#ifndef SHELL_NO_ECHO
            _putchar('\r');
            _putchar('\n');
#endif

            /* return 1 if line is empty, 0 otherwise */
            return line_buf_ptr == buf;
        }
        /* QEMU uses 0x7f (DEL) as backspace, while 0x08 (BS) is for most terminals */
        else if (c == 0x08 || c == 0x7f) {
            puts("pot \r\n");
            if (line_buf_ptr == buf) {
                /* The line is empty. */
                continue;
            }
            //puts("shell qemu stuff \r\n");
            *--line_buf_ptr = '\0';
            /* white-tape the character */
#ifndef SHELL_NO_ECHO
            _putchar('\b');
            _putchar(' ');
            _putchar('\b');
#endif
        }
        else {
            *line_buf_ptr++ = c;
#ifndef SHELL_NO_ECHO
            _putchar(c);
#endif
        }
    }
}
static int readline_costum(char *buf, size_t size)
{
    char *line_buf_ptr = buf;
    int x_count = 0;
    int line_length=0;
    while (1) {
        line_length ++;
        if ((line_buf_ptr - buf) >= ((int) size) - 1) {
            return -1;
        }

        int c = getchar();
        if (c < 0) {
            return 1;
        }
        if( c=='x'){
            x_count ++;
            if(x_count == 5){
                *line_buf_ptr = '\0';
                printf("\r\n riot read line  %d \r\n",line_length);
                return line_buf_ptr == buf;
            }
        }else{
            x_count = 0;
        }
        *line_buf_ptr++ = c;
    }
}
static void handle_input_line_costum(const shell_command_t *command_list, char *line)
{
    //static const char *INCORRECT_QUOTING = "shell: incorrect quoting";

    /* first we need to calculate the number of arguments */
    unsigned argc = 5;
    char *pos = line;
    //int lastArg= 0;
    int x_count = 0;
    int quate_open = 0;
    int counter =0;
    int args = 0;
    while ((x_count < 4)&&(args < 4)) {
        if(*pos == 'x'){
            x_count ++;
        }else{
            x_count = 0;
        }
        if(*pos == '"'){
            if(quate_open) {
                quate_open = 0;
                break;
            }
            else
                quate_open = 1;
        }
        if((*pos == ' ') &&(quate_open == 0) ){
            *pos = 0;
            //printf("nullify the line \r\n");
            args++;
        }
        counter ++;
        ++pos;
    }
    //printf("handle line %d \r\n",counter);
    /* then we fill the argv array */
    char *argv[argc + 1];
    argv[argc] = NULL;
    pos = line;
    for (unsigned i = 0; i < argc; ++i) {
        while (!*pos) {
            ++pos;
        }
        if (*pos == '"') {
            ++pos;
        }
        argv[i] = pos;
        while (*pos) {
            ++pos;
        }
    }
    //printf("agrv 4 len %d \r\n",strlen(argv[4]));
    //char starter[1];
    //starter[0] = '"';
    //argv[3] = strstr(line,starter);
    /* then we call the appropriate handler */
    shell_command_handler_t handler = find_handler(command_list, argv[0]);
    if (handler != NULL) {
        handler(argc, argv);
    }
    else {
        if (strcmp("help", argv[0]) == 0) {
            print_help(command_list);
        }
        else {
            printf("shell: command not found: %s\n", argv[0]);
        }
    }
}


static inline void print_prompt(void)
{
#ifndef SHELL_NO_PROMPT
    _putchar('>');
    _putchar(' ');
#endif

#ifdef MODULE_NEWLIB
    fflush(stdout);
#endif
}

void shell_run(const shell_command_t *shell_commands, char *line_buf, int len)
{
    print_prompt();

    while (1) {

        int res = readline(line_buf, len);
        if (!res) {
            handle_input_line(shell_commands, line_buf);
        }

        print_prompt();
    }
}
void shell_run_costum(const shell_command_t *shell_commands, char *line_buf, int len)
{
    print_prompt();

    while (1) {
        //puts("shell reading line \r\n");
        //int res = readline(line_buf, len);
        int res = readline_costum(line_buf, len);
        //puts("shell just read new line \r\n");
        if (!res) {
            handle_input_line_costum(shell_commands, line_buf);
            //puts("new line handled in shell \r\n");
        }

        print_prompt();
    }
}
