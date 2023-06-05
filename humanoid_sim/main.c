#include <stdio.h>
#include <time.h>
#include <string.h>
#include "mujoco.h"
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "humanoidmujoco.h"
#include "udp.h"

//related to writing data to a file
FILE *fid;
int loop_index = 0;
const int data_frequency = 10; //frequency at which data is written to a file
char path[] = "../";
char datafile[] = "data.csv";

static long long get_microseconds(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000000 + now.tv_nsec / 1000;
}
//****************************
//This function is called once and is used to get the headers
void init_save_data()
{
    //write name of the variable here (header)
    fprintf(fid,"t     ,");
    fprintf(fid,"target_q1    ,");
    fprintf(fid,"q1           ,");
    fprintf(fid,"left_foot_fx    ,");
    fprintf(fid,"left_foot_fy    ,");
    fprintf(fid,"left_foot_fz    ,");

    //Don't remove the newline
    fprintf(fid,"\n");
}


int main() {
    char datapath[100]={};
    strcat(datapath,path);
    strcat(datapath,datafile);
    // Create cassie simulation
    const char modelfile[] = "../humanoid.xml";
    //robot_sim_t *sim = robot_sim_init(modelfile, false);
    robot_sim_t *sim = robot_sim_init(modelfile, false);
    robot_vis_t *vis;

    vis = robot_vis_init(sim, modelfile);

    //UDP init
    bool run_sim = false;

    struct sockaddr_storage src_addr;
    socklen_t addrlen = sizeof src_addr;

    char *iface_addr_str = "0.0.0.0";
    char *iface_port_str = "10001";

    int dinlen, doutlen;
    dinlen = 8;
    doutlen = 8;
    const int recvlen = dinlen;
    const int sendlen = doutlen;
    unsigned char *recvbuf = malloc(recvlen);
    unsigned char *sendbuf = malloc(sendlen);

    const unsigned char *data_in = recvbuf;
    unsigned char *data_out = sendbuf;

    robot_state state_out;
    robot_ctrl ctrl_in;


    int sock = udp_init_host(iface_addr_str, iface_port_str);
    printf("server: \n ");

    // Manage simulation loop
    unsigned long long loop_counter = 0;

    double cfrc[12];
    double p_gain_arm[6];
    double d_gain_arm[6];
    double p_gain_leg[10];
    double d_gain_leg[10];
    double p_gain[16];
    double d_gain[16];
    // leg first
    double target_q[16] = {0, 0, 0., 0., 0,   //right leg
                           0, 0, 0., 0., 0,   //left leg
                           0,0,0,  //right arm
                           0,0,0}; //left arm
    for (int i = 0; i<10; i++)
    {
        p_gain_leg[i] = 10;
        d_gain_leg[i] = 0.1;
    }
    for (int i = 0; i<6; i++)
    {
        p_gain_arm[i] = 10;
        d_gain_arm[i] = 0.1;
    }
    for (int i = 0; i<16; i++)
    {
        p_gain[i] = 1200;
        d_gain[i] = 0.02;
    }
    // define sim data
    double* base_pos;

    fid = fopen(datapath,"w");
    init_save_data();
    bool render_state = true;
    mj_main_init(sim);
    body_sim_hold(sim);
    while (render_state) {
        // get UDP data
        ssize_t nbytes;
        nbytes = get_newest_packet(sock, recvbuf, recvlen,(struct sockaddr *) &src_addr, &addrlen);
        if (recvlen == nbytes)
        {
            unpack_ctrl_in(data_in, &ctrl_in);
            run_sim = true;
        }
        // get sim time
        size_t compute_start = get_microseconds();
        double sim_start     = *robot_sim_time(sim);
        //step(sim);
        // run sim
        if (run_sim)
        {
            target_q[2] = ctrl_in.robot_ctrl_data.target_q;  //for udp test
            robot_sim_set_pd_control(sim, target_q, p_gain, d_gain);
            //robot_sim_foot_forces(sim, cfrc);
            //printf("left foot =  %f   right foot = %f \n", cfrc[2], cfrc[8]);
            robot_step(sim, &state_out);

            // collect sim data

//            base_pos = robot_sim_xpos(sim, "torso");
//            robot_sim_data.sim_data.base_info.xyz[0] = base_pos[0];
//            robot_sim_data.sim_data.base_info.xyz[1] = base_pos[1];
//            robot_sim_data.sim_data.base_info.xyz[2] = base_pos[2];
            //robot_sim_data.sim_data.sim_time = *robot_sim_time(sim);
            pack_state_out(&state_out, data_out);
            send_packet(sock, sendbuf, sendlen, (struct sockaddr *) &src_addr, addrlen);
        }
        save_data(fid, sim, cfrc);
        // sim time = cpu time
        double cpu_deltat, sim_deltat;
        double sim_end = *robot_sim_time(sim);
        do{
            size_t compute_end = get_microseconds();
            cpu_deltat  = (double)(compute_end - compute_start) / 1e6;
            sim_deltat  = sim_end - sim_start;
        } while(sim_deltat > cpu_deltat);
        if(cpu_deltat > sim_deltat + 1e-4){
            printf("SLOWER THAN REAL TIME BY %6.5fs\n", cpu_deltat - sim_deltat);

        }

        if (loop_counter % 33 == 0)
            render_state = robot_vis_draw(vis, sim);

        // Increment loop counter
        ++loop_counter;

    }

    return 0;
}

