/*
 * Copyright (c) 2018 Sam Kumar
 * Copyright (c) 2018 University of California, Berkeley
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * This is the Stream TO Message Protocol daemon, which receives a stream of
 * bytes in one unix socket (e.g., @stomp) and writes those bytes, chunked into
 * messages, to an output unix socket. The output unix socket could be a
 * channel in REthos.
 */

#include <assert.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>

#define BUF_LEN 200
char buffer[BUF_LEN];

FILE* msglog = NULL;

static void check_fatal_error(const char* msg) {
    assert(errno);
    if (msglog != NULL) {
        fprintf(msglog, "%s: %s\n", msg, strerror(errno));
        fflush(msglog);
    }
    exit(1);
}

static int socket_un_create(void) {
    int dsock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (dsock == -1) {
        check_fatal_error("Could not create domain socket");
    }
    int flags = fcntl(dsock, F_GETFL);
    if (flags == -1) {
        check_fatal_error("Could not get socket flags");
    }
    flags = fcntl(dsock, F_SETFL, flags | O_NONBLOCK);
    if (flags == -1) {
        check_fatal_error("Could not set socket flags");
    }
    return dsock;
}

static size_t sockaddr_un_fill(struct sockaddr_un* addr, const char* name) {
    addr->sun_family = AF_UNIX;
    addr->sun_path[0] = '\0';
    strncpy(&addr->sun_path[1], name, sizeof(addr->sun_path) - 1);
    return strlen(name) + sizeof(addr->sun_family) + 1;
}

static void checked_write(int fd, const void* buffer, size_t size) {
    const char* buf = buffer;
    size_t written = 0;
    while (written < size) {
        ssize_t rv = write(fd, &buf[written], size - written);
        if (rv == -1) {
            char errbuf[50];
            snprintf(errbuf, sizeof(errbuf), "write to fd %d failed", fd);
            check_fatal_error(errbuf);
        }
        written += rv;
    }
}

int main(int argc, char** argv) {
    if (argc != 2 && argc != 3) {
        printf("Usage: %s <message socket name> [<serial socket name>]\n", argv[0]);
        return 1;
    }

    char* logpath = getenv("STOMPLOG");
    if (logpath != NULL) {
        msglog = fopen("/home/ubuntu/stomp.log", "w+");
        if (msglog == NULL) {
            perror("Could not open log file");
            return 2;
        }
    }

    int lsock = -1;
    int ssock = -1;

    /* Always the same as ssock, except when ssock is STDIN_FILENO. */
    int ssock_output = -1;

    struct sockaddr_un addr;
    size_t addr_len;

    if (argc == 3) {
        const char* serial_socket_name = argv[2];

        if (msglog != NULL) {
            fprintf(msglog, "Listening on %s...\n", serial_socket_name);
            fflush(msglog);
        }
        lsock = socket_un_create();
        addr_len = sockaddr_un_fill(&addr, serial_socket_name);
        if (bind(lsock, (struct sockaddr*) &addr, addr_len) == -1) {
            check_fatal_error("Could not bind serial listen socket");
        }
        if (listen(lsock, 1) == -1) {
            check_fatal_error("Could not listen on serial listen socket");
        }
    } else {
        if (msglog != NULL) {
            fprintf(msglog, "No serial socket provided; using stdin/stdout\n");
            fflush(msglog);
        }
        ssock = STDIN_FILENO;
        ssock_output = STDOUT_FILENO;
    }

    const char* message_socket_name = argv[1];
    if (msglog != NULL) {
        fprintf(msglog, "Connecting to %s...\n", message_socket_name);
    }
    int msock = socket_un_create();
    addr_len = sockaddr_un_fill(&addr, message_socket_name);
    if (connect(msock, (struct sockaddr*) &addr, addr_len) == -1) {
        check_fatal_error("Could not connect message socket");
    }
    if (msglog != NULL) {
        fprintf(msglog, "Done connecting to %s.\n", message_socket_name);
        fflush(msglog);
    }

    /* State variables for reading messages. */
    int message_header_bytes_left = 4;
    uint32_t message_body_bytes_left = 0;

    for (;;) {
        int first_fd = msock;
        int second_fd = (ssock == -1) ? lsock : ssock;
        int max_fd = (first_fd < second_fd) ? second_fd : first_fd;

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(first_fd, &read_fds);
        FD_SET(second_fd, &read_fds);

        int rv = select(max_fd + 1, &read_fds, NULL, NULL, NULL);
        if (rv == -1) {
            check_fatal_error("Could not wait for event");
        }

        if (FD_ISSET(msock, &read_fds)) {
            // Transfer message from msock to isock as bytes
            if (message_header_bytes_left != 0) {
                ssize_t bytes_read = read(msock, ((uint8_t*) &message_body_bytes_left) + (4 - message_header_bytes_left), message_header_bytes_left);
                if (bytes_read == -1) {
                    check_fatal_error("Could not read from message socket");
                } else if (bytes_read == 0) {
                    if (msglog != NULL) {
                        fprintf(msglog, "Message socket closed\n");
                        fflush(msglog);
                    }
                    return 0;
                } else {
                    message_header_bytes_left -= bytes_read;
                    if (message_header_bytes_left == 0) {
                        message_body_bytes_left = be32toh(message_body_bytes_left);
                        if (msglog != NULL) {
                            fprintf(msglog, "Got a message of length %u\n", (unsigned int) message_body_bytes_left);
                        }
                    }
                    if (message_body_bytes_left == 0) {
                        message_header_bytes_left = 4;
                    }
                }
            } else {
                ssize_t bytes_read = read(msock, buffer, message_body_bytes_left < BUF_LEN ? message_body_bytes_left : BUF_LEN);
                if (bytes_read == -1) {
                    check_fatal_error("Could not read from message socket");
                } else if (bytes_read == 0) {
                    if (msglog != NULL) {
                        fprintf(msglog, "Message socket closed\n");
                    }
                    return 0;
                } else {
                    if (ssock != -1) {
                        checked_write(ssock_output, buffer, bytes_read);
                    }
                    if (msglog != NULL) {
                        fprintf(msglog, "Transferred %d bytes to ssock_output\n", (int) bytes_read);
                    }
                    message_body_bytes_left -= bytes_read;
                    if (message_body_bytes_left == 0) {
                        message_header_bytes_left = 4;
                    }
                }
            }
        }

        if (ssock == -1) {
            if (FD_ISSET(lsock, &read_fds)) {
                // Accept incoming connection
                ssock = accept(lsock, NULL, NULL);
                if (ssock == -1) {
                    check_fatal_error("Could not accept serial connection");
                }
                ssock_output = ssock;
            }
        } else if (FD_ISSET(ssock, &read_fds)) {
            // Transfer bytes from ssock to msock, chunked as a message
            ssize_t bytes_read = read(ssock, buffer, BUF_LEN);
            if (bytes_read == -1) {
                check_fatal_error("Could not read from serial socket");
            } else if (bytes_read == 0) {
                int rv = close(ssock);
                if (rv == -1) {
                    check_fatal_error("Could not close serial socket");
                }
                ssock = -1;
                if (lsock == -1) {
                    if (msglog != NULL) {
                        fprintf(msglog, "Stdin was closed\n");
                        fflush(msglog);
                    }
                    return 0;
                }
            } else {
                uint32_t header = (uint32_t) bytes_read;
                header = htobe32(header);
                checked_write(msock, &header, sizeof(header));
                checked_write(msock, buffer, bytes_read);
            }
        }

        if (msglog != NULL) {
            fflush(msglog);
        }
    }

    return 0;
}
