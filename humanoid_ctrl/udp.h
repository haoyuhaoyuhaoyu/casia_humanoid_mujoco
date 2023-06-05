#ifndef UDP_H
#define UDP_H

#define PACKET_HEADER_LEN 2

// Data and results for processing packet header
typedef struct {
    char seq_num_out;
    char seq_num_in_last;
    char delay;
    char seq_num_in_diff;
} packet_header_info_t;

typedef struct {
    double sim_time;
} sim_data;

typedef struct {
    sim_data robot_sim_data;
} robot_state;

typedef struct {
    double target_q;
} ctrl_data;

typedef struct {
    ctrl_data robot_ctrl_data;
} robot_ctrl;

#ifdef __cplusplus
extern "C" {
#endif

// Process packet header used to measure delay and skipped packets
void process_packet_header(packet_header_info_t *info,
                           const unsigned char *header_in,
                           unsigned char *header_out);

#include <sys/socket.h>

// Create a UDP socket listening at a specific address/port
int udp_init_host(const char *addr_str, const char *port_str);

// Create a UDP socket connected and listening to specific addresses/ports
int udp_init_client(const char *remote_addr_str, const char *remote_port_str,
                    const char *local_addr_str, const char *local_port_str);

// Close a UDP socket
void udp_close(int sock);

// Get newest valid packet in RX buffer
ssize_t get_newest_packet(int sock, void *recvbuf, size_t recvlen,
                          struct sockaddr *src_addr, socklen_t *addrlen);

// Wait for a new valid packet
ssize_t wait_for_packet(int sock, void *recvbuf, size_t recvlen,
                        struct sockaddr *src_addr, socklen_t *addrlen);

// Send a packet
ssize_t send_packet(int sock, void *sendbuf, size_t sendlen,
                    struct sockaddr *dst_addr, socklen_t addrlen);

// pack and unpack data
void unpack_state_in(const unsigned char *bytes, robot_state *bus);
void pack_ctrl_out(const robot_ctrl *bus, unsigned char *bytes);

#ifdef __cplusplus
}
#endif

#endif // UDP_H
