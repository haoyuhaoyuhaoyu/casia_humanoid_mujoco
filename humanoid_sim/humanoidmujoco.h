//
// Created by han on 23-2-1.
//

#ifndef AMBER_B1_MUJOCO_AMBERB1MUJOCO_H
#define AMBER_B1_MUJOCO_AMBERB1MUJOCO_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/socket.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct robot_sim robot_sim_t;
typedef struct robot_vis robot_vis_t;

#define STATE_OUT_T_PACKED_LEN 200
#define PD_IN_T_PACKED_LEN 240

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

// if via union
union ROBOT_SIM_DATA{
    struct{
        double sim_time;
    }sim_data;
    u_int8_t buffer[8]; //double = 8 * u_int8_t
};

union ROBOT_CTRL_DATA{
    struct{
        double target_q;
    }ctrl_data;
    u_int8_t buffer[8]; //double = 8*u_int8_t
};

bool robot_mujoco_init(const char *modelfile);

/*******************************************************************************
* robot simulator functions
******************************************************************************/

robot_sim_t *robot_sim_init(const char *modelfile, bool reinit);

double *robot_sim_time(robot_sim_t *sim);


/*******************************************************************************
* robot visualizer functions
******************************************************************************/

robot_vis_t *robot_vis_init(robot_sim_t *sim, const char* modelfile);

void robot_vis_close(robot_vis_t *vis);

bool robot_vis_draw(robot_vis_t *vis, robot_sim_t *sim);

/*******************************************************************************
* robot control functions
******************************************************************************/

void pack_state_out(const robot_state *bus, unsigned char *bytes);
void unpack_ctrl_in(const unsigned char *bytes, robot_ctrl *bus);

void robot_step(robot_sim_t *sim, robot_state *bus);
// utils
void tt();
double* robot_sim_xquat(robot_sim_t *c, const char* name);
double* robot_sim_xpos(robot_sim_t *c, const char* name);

void body_sim_fixed(robot_sim_t *sim);
int mj_main_init(robot_sim_t *sim);
void step(robot_sim_t *sim);
void body_sim_hold(robot_sim_t *sim);
void robot_sim_set_pd_control(robot_sim_t *sim, double target_pos[16], double p_gain[6], double d_gain[6]);
void robot_sim_foot_forces(const robot_sim_t *sim, double cfrc[12]);
void save_data(FILE *fid, robot_sim_t *c, double cfrc[12]);

#ifdef __cplusplus
}
#endif


#endif //AMBER_B1_MUJOCO_AMBERB1MUJOCO_H
