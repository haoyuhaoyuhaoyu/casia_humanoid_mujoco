#include <iostream>
#include <unistd.h>
#include "cmath"
#include "udp.h"
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "string.h"

#include <chrono>
#include <memory>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"

static long long get_microseconds(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000000 + now.tv_nsec / 1000;
}

int main() 
{
    char *remote_addr_str = "127.0.0.1"; //use ifconfig
    char *remote_port_str = "10001";

    struct sockaddr_storage src_addr = {0};
    socklen_t addrlen = sizeof src_addr;

    char *iface_addr_str = "0.0.0.0";
    char *iface_port_str = "10000";

    robot_state state_in;
    robot_ctrl ctrl_out;

    int dinlen, doutlen;
    dinlen = 8;
    doutlen = 8;
    const int recvlen = dinlen;
    const int sendlen = doutlen;
    auto *recvbuf = static_cast<unsigned char *>(malloc(recvlen));
    auto *sendbuf = static_cast<unsigned char *>(malloc(sendlen));

    const unsigned char *data_in = recvbuf;
    unsigned char *data_out = sendbuf;

    int sock = udp_init_client(remote_addr_str, remote_port_str, iface_addr_str, iface_port_str);
    std::cout<<"client:"<<std::endl;

    bool received_data = false;
    double sim_time;
    memset(sendbuf, 0, sendlen);
    printf("Connecting to sim ...\n");

    while(true)
    {
        if (!received_data) {
            // Send null commands until the simulator responds
            ssize_t nbytes;
            do {
                send_packet(sock, sendbuf, sendlen, NULL, NULL);
                usleep(1000);
                nbytes = get_newest_packet(sock, recvbuf, recvlen, NULL, NULL);
            } while (recvlen != nbytes);
            received_data = true;
            printf("Connected!\n\n");
        } else {
            // Wait for a new packet
            wait_for_packet(sock, recvbuf, recvlen, NULL, NULL);
            printf("receive UDP data = %f \n", state_in.robot_sim_data.sim_time);
        }
        unpack_state_in(data_in, &state_in);

        //log cpu start
        size_t compute_start = get_microseconds();

        // run controller
        sim_time = state_in.robot_sim_data.sim_time;
        ctrl_out.robot_ctrl_data.target_q = sin(sim_time);
        //

        double cpu_deltat;
        do{
            size_t compute_end = get_microseconds();
            cpu_deltat  = (double)(compute_end - compute_start) / 1e3; //convert microsecond to millisecond
            //printf("cpu time = %f", cpu_deltat);
        } while( 1 > cpu_deltat);
        if(cpu_deltat > 1 + 1e-4){
            printf("SLOWER THAN REAL TIME BY %6.5f ms\n", cpu_deltat - 1);

        }
        pack_ctrl_out(&ctrl_out, data_out);
        send_packet(sock, sendbuf, sendlen, NULL, NULL);

    }
    return 0;
}
