//
// Created by han on 23-2-1.
//

#include "humanoidmujoco.h"
#include "mujoco.h"

#include "glfw3.h"

#include <pwd.h>
#include <dlfcn.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


// Linux sockets
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <netdb.h>
#include <fcntl.h>

#define SOCKETS_INIT
#define SOCKETS_CLEANUP
typedef int ioctl_arg_t;


/*******************************************************************************
 * Global library state
 ******************************************************************************/

static bool glfw_initialized = false;
static bool mujoco_initialized = false;
static mjModel *initial_model;
static int left_foot_body_id;
static int right_foot_body_id;
// Globals for visualization
static int fontscale = mjFONTSCALE_200;
//static unsigned char *frame;
mjvFigure figsensor;

union byte8{
    double fdata;
    uint8_t byte[8];
};

#define MAX_VIS_MARKERS 500

struct robot_sim {
    mjModel *m;
    mjData *d;
};

struct vis_marker_info {
    int id;

    double pos_x;
    double pos_y;
    double pos_z;

    double size_x;
    double size_y;
    double size_z;

    double r;
    double g;
    double b;
    double a;

    double so3[9];
};

struct robot_vis {
    //visual interaction controls
    double lastx;
    double lasty;
    bool button_left;
    bool button_middle;
    bool button_right;

    int lastbutton;
    double lastclicktm;

    int refreshrate;

    int showhelp;
    bool showoption;
    bool showGRF;
    int GRFcount;
    bool showfullscreen;
    bool showsensor;
    bool slowmotion;

    bool showinfo;
    bool paused;

    int framenum;
    int lastframenum;

    int perturb_body;   // Body to apply perturb force to in vis_draw
    double perturb_force[6];    // Perturb force to apply

    // Markers
    size_t marker_num;
    struct vis_marker_info marker_infos[MAX_VIS_MARKERS];

    // GLFW  handle
    GLFWwindow *window;

    // File Recording Stuff
    FILE *pipe_video_out;
    int video_width;
    int video_height;

    // MuJoCo stuff

    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    mjvPerturb pert;
    mjvFigure figsensor;
    mjvFigure figGRF;
    mjModel* m;
    mjData* d;
};

/*******************************************************************************
 * Dynamic library loading
 ******************************************************************************/

// Loaded MuJoCo functions
#define MUJOCO_FUNCTION_LIST                    \
    X(mj_activate)                              \
    X(mj_deactivate)                            \
    X(mj_loadXML)                               \
    X(mj_copyModel)                             \
    X(mj_deleteModel)                           \
    X(mj_makeData)                              \
    X(mj_copyData)                              \
    X(mj_deleteData)                            \
    X(mj_resetData)                             \
    X(mj_forward)                               \
    X(mj_setConst)                              \
    X(mj_fwdPosition)                           \
    X(mj_comVel)                                \
    X(mj_step1)                                 \
    X(mj_step2)                                 \
    X(mj_step)                                  \
    X(mj_contactForce)                          \
    X(mj_name2id)                               \
    X(mj_id2name)                               \
    X(mj_fullM)                                 \
    X(mju_copy)                                 \
    X(mju_zero)                                 \
    X(mju_rotVecMatT)                           \
    X(mju_rotVecQuat)                           \
    X(mju_sub3)                                 \
    X(mju_mulMatTVec)                           \
    X(mju_mat2Quat)                             \
    X(mju_printMat)                             \
    X(mjv_makeScene)                            \
    X(mjv_defaultScene)                         \
    X(mjv_freeScene)                            \
    X(mjv_updateScene)                          \
    X(mjv_defaultCamera)                        \
    X(mjv_defaultOption)                        \
    X(mjv_movePerturb)                          \
    X(mjv_moveCamera)                           \
    X(mjv_initPerturb)                          \
    X(mjv_select)                               \
    X(mjv_applyPerturbForce)                    \
    X(mjv_applyPerturbPose)                     \
    X(mjr_defaultContext)                       \
    X(mjv_defaultFigure)                        \
    X(mjr_makeContext)                          \
    X(mjr_freeContext)                          \
    X(mjr_render)                               \
    X(mjr_overlay)                              \
    X(mjr_figure)                               \
    X(mjr_readPixels)

// Loaded GLFW functions
#define GLFW_FUNCTION_LIST                      \
    X(glfwInit)                                 \
    X(glfwTerminate)                            \
    X(glfwCreateWindow)                         \
    X(glfwDestroyWindow)                        \
    X(glfwMakeContextCurrent)                   \
    X(glfwGetWindowUserPointer)                 \
    X(glfwSetWindowUserPointer)                 \
    X(glfwSetWindowCloseCallback)               \
    X(glfwSetCursorPosCallback)                 \
    X(glfwSetMouseButtonCallback)               \
    X(glfwSetScrollCallback)                    \
    X(glfwSetKeyCallback)                       \
    X(glfwGetFramebufferSize)                   \
    X(glfwSwapBuffers)                          \
    X(glfwSwapInterval)                         \
    X(glfwPollEvents)                           \
    X(glfwGetVideoMode)                         \
    X(glfwGetPrimaryMonitor)                    \
    X(glfwGetWindowSize)                        \
    X(glfwGetKey)                               \
    X(glfwGetMouseButton)                       \
    X(glfwGetCursorPos)                         \
    X(glfwRestoreWindow)                        \
    X(glfwMaximizeWindow)                       \
    X(glfwSetWindowShouldClose)                 \
    X(glfwWindowShouldClose)                    \
    X(glfwSetWindowSize)

// Dynamic object handles
static void *mj_handle;
static void *glfw_handle;
static void *gl_handle;
static void *glew_handle;

// Function pointers
#define X(fun)                                  \
    typedef __typeof__(fun) fun ## _fp_type;    \
    static fun ## _fp_type *fun ## _fp;
MUJOCO_FUNCTION_LIST
GLFW_FUNCTION_LIST
#undef X

#define LOADLIB(path) dlopen(path, RTLD_LAZY | RTLD_GLOBAL)
#define UNLOADLIB(handle) dlclose(handle)
#define LOADFUN(handle, sym) sym ## _fp = dlsym(handle, #sym)
#define MJLIBNAME "libmujoco210.so"
#define MJLIBNAMENOGL "libmujoco210nogl.so"
#define GLFWLIBNAME "libglfw.so.3"

#define AMBERB1_SIM_ALLOC_POINTER(c)                 \
    do {                                        \
        c->d = mj_makeData_fp(c->m);   \
    } while (0)

#define CASSIE_FREE_POINTER(c)                  \
    do {                                        \
        mj_deleteData_fp(c->d);                 \
    } while (0)


// Redefine mjVISSTRING HAAAACKKKK to fix stupid bug
const char* VISSTRING[mjNVISFLAG][3] = { {"Convex Hull"    ,"0",  "H"},
                                         {"Texture"         ,"1",  "X"},
                                         {"Joint"           ,"0",  "J"},
                                         {"Actuator"        ,"0",  "U"},
                                         {"Camera"          ,"0",  "Q"},
                                         {"Light"           ,"0",  "Z"},
                                         {"Tendon"          ,"1",  "V"},
                                         {"Range Finder"    ,"1",  "Y"},
                                         {"Constraint"      ,"0",  "N"},
                                         {"Inertia"         ,"0",  "I"},
                                         {"SCL Inertia"     ,"0",  "S"},
                                         {"Perturb Force"   ,"0",  "B"},
                                         {"Perturb Object"  ,"1",  "O"},
                                         {"Contact Point"   ,"0",  "C"},
                                         {"Contact Force"   ,"0",  "F"},
                                         {"Contact Split"   ,"0",  "P"},
                                         {"Transparent"     ,"0",  "T"},
                                         {"Auto Connect"    ,"0",  "A"},
                                         {"Center of Mass"  ,"0",  "M"},
                                         {"Select Point"    ,"0",  "E"},
                                         {"Static Body"     ,"1",  "D"},
                                         {"Skin"            ,"1",  ";"}};
const char* RNDSTRING[mjNRNDFLAG][3] = {{"Shadow"      ,"1",  "S"},
                                        {"Wireframe"   ,"0",  "W"},
                                        {"Reflection"  ,"1",  "R"},
                                        {"Additive"    ,"0",  "L"},
                                        {"Skybox"      ,"1",  "K"},
                                        {"Fog"         ,"0",  "G"},
                                        {"Haze"        ,"1",  "/"},
                                        {"Segment"     ,"0",  ","},
                                        {"Id Color"    ,"0",  "."}};
/*******************************************************************************
 * Public functions
 ******************************************************************************/
double TARGET_q[16];


#define ID_NAME_LOOKUP(model, idvar, objtype, name)                            \
    do {                                                                \
        idvar = mj_name2id_fp(model, objtype, #name);                   \
        if (-1 == idvar) {                                              \
            fprintf(stderr, "Could not find body named " #name "\n");   \
            return false;                                               \
        }                                                               \
    } while (0)

static void window_close_callback(GLFWwindow *window)
{
    robot_vis_close(glfwGetWindowUserPointer_fp(window));
}


robot_sim_t *robot_sim_init(const char* modelfile, bool reinit)
{
    // Make sure MuJoCo is initialized and the model is loaded
    if (!mujoco_initialized) {
        if (!robot_mujoco_init(modelfile)) {
            return NULL;
        }
    }

    // Allocate memory, zeroed for cassie_out_t and filter initialization
    robot_sim_t *c = calloc(1, sizeof (robot_sim_t));

    if (reinit) {
        // pass
    } else {
        // Initialize mjModel
        c->m = mj_copyModel_fp(NULL, initial_model);
    }

    // Allocate pointer types
    AMBERB1_SIM_ALLOC_POINTER(c);

    // Set initial joint configuration
    double qpos_init[] = {-0.04892201261633587, 0.137070621911611, 0.4259146556246555,
                          0.9947601, -0.0483187, -0.0857336, 0.0277026,
                          0.1324580211741131, 0.8517430801704898, -0.9228175102197048,
                          0.07249285764925784, 0.5326225286226256, -0.8761352725538367};
    mju_copy_fp(c->d->qpos, qpos_init, 13);
    mj_forward_fp(c->m, c->d);

    return c;
}

static bool load_glfw_library(const char *basedir)
{
    // Buffer for paths
    char buf[4096 + 1024];

#ifndef _WIN32
    // Open dependencies
    gl_handle = LOADLIB("libGL.so.1");
    snprintf(buf, sizeof buf, "%.4096s/.mujoco/mujoco210/bin/libglew.so", basedir);
    glew_handle = LOADLIB(buf);
    if (!gl_handle || !glew_handle) {
        printf("gl_handle or glew_handle not loaded\n");
        return false;
    }
#endif

    // Open library
    snprintf(buf, sizeof buf, "%.4096s/.mujoco/mujoco210/bin/" GLFWLIBNAME, basedir);
    glfw_handle = LOADLIB(buf);
    if (!glfw_handle) {
        //fprintf(stderr, "Failed to load %s\n", buf);
        return false;
    }

        // Get function pointers
#define X(fun) LOADFUN(glfw_handle, fun);
    GLFW_FUNCTION_LIST
#undef X

    return true;
}


static bool load_mujoco_library()
{
    // Buffer for paths
    char buf[4096 + 1024];
    // Get home directory
    const char* homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    // Try loading GLFW
    bool __attribute__((unused)) gl = load_glfw_library(homedir);
    //struct passwd *pw = getpwuid(getuid());

    // Choose library version
    snprintf(buf, sizeof buf, "%.4096s/.mujoco/mujoco210/bin/" MJLIBNAME, homedir);
    // snprintf(buf, sizeof buf, "%.4096s/mujoco200_linux/bin/" MJLIBNAME, "~/.mujoco");
    if (!gl) {
        snprintf(buf, sizeof buf, "%.4096s/.mujoco/mujoco210/bin/" MJLIBNAMENOGL, homedir);
        // snprintf(buf, sizeof buf, "%.4096s/.mujoco/mujoco200_linux/bin/" MJLIBNAMENOGL, "~");
    }

    // Open library
    mj_handle = LOADLIB(buf);
    if (!mj_handle) {
        fprintf(stderr, "Failed to load %s\n%s\n", buf, dlerror());
        return false;
    }

        // Get function pointers
#define X(fun) LOADFUN(mj_handle, fun);
    MUJOCO_FUNCTION_LIST
#undef X
    return true;
}

bool robot_mujoco_init(const char *file_input)
{
    // Check if mujoco has already been initialized
    if (!mujoco_initialized) {
        char binpath[4096];
        if (-1 == readlink("/proc/self/exe", binpath, sizeof binpath))
            fprintf(stderr, "Failed to get binary directory\n");

        // Load MuJoCo
        if (!load_mujoco_library()) {
            return false;
        }

        // Activate MuJoCo
        const char* key_buf = getenv("MUJOCO_KEY_PATH");
        mj_activate_fp(key_buf);

        const char* modelfile;
        modelfile = file_input;

        char error[1000] = "Could not load XML model";
        initial_model = mj_loadXML_fp(modelfile, 0, error, 1000);
        if (!initial_model) {
            printf("%s\n", error);
            return false;
        }

        mujoco_initialized = true;

    }

    // Initialize GLFW if it was loaded
    if (glfw_handle && !glfw_initialized) {
        if (!glfwInit_fp()) {
            return false;
        }
        glfw_initialized = true;
    }
    return mujoco_initialized;
}

double *robot_sim_time(robot_sim_t *c)
{
    return &c->d->time;
}

void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    (void)xoffset;
    robot_vis_t* v = glfwGetWindowUserPointer_fp(window);
    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera_fp(v->m, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &v->scn, &v->cam);
}

void mouse_move(GLFWwindow* w, double xpos, double ypos) {
    robot_vis_t* v = glfwGetWindowUserPointer_fp(w);

    // no buttons down: nothing to do
    if (!v->button_left && !v->button_middle && !v->button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - v->lastx;
    double dy = ypos - v->lasty;
    v->lastx = xpos;
    v->lasty = ypos;

    int width;
    int height;
    glfwGetWindowSize_fp(w, &width, &height);

    int mod_shift = glfwGetKey_fp(w, GLFW_KEY_LEFT_SHIFT) || glfwGetKey_fp(w, GLFW_KEY_RIGHT_SHIFT);

    // determine action based on mouse button
    int action = mjMOUSE_ZOOM;
    if (v->button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (v->button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }

    // move perturb or camera
    mjtNum xchange = dx / height;
    mjtNum ychange = dy / height;
    if (v->pert.active != 0) {
        mjv_movePerturb_fp(v->m, v->d, action, xchange, ychange, &v->scn, &v->pert);
    } else {
        mjv_moveCamera_fp(v->m, action, xchange, ychange, &v->scn, &v->cam);
    }
}

// past data for double-click detection
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    robot_vis_t* v = glfwGetWindowUserPointer_fp(window);
    // update button state
    v->button_left = glfwGetMouseButton_fp(window, GLFW_MOUSE_BUTTON_LEFT);
    v->button_middle = glfwGetMouseButton_fp(window, GLFW_MOUSE_BUTTON_MIDDLE);
    v->button_right = glfwGetMouseButton_fp(window, GLFW_MOUSE_BUTTON_RIGHT);

    // Alt: swap left and right
    if (mods == GLFW_MOD_ALT) {
        bool tmp = v->button_left;
        v->button_left = v->button_right;
        v->button_right = tmp;

        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            button = GLFW_MOUSE_BUTTON_RIGHT;
        } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            button = GLFW_MOUSE_BUTTON_LEFT;
        }
    }

    // update mouse position
    double x, y;
    glfwGetCursorPos_fp(window, &x, &y);
    v->lastx = x;
    v->lasty = y;

    // set perturbation
    int newperturb = 0;
    if (mods == GLFW_MOD_CONTROL && v->pert.select > 0) {
        if (act == GLFW_PRESS) {
            // Disable vis perturb force when using mouse perturb, only want to vis perturb object
            v->opt.flags[11] = 0;
            // right: translate;  left: rotate
            if (v->button_right) {
                newperturb = mjPERT_TRANSLATE;
            } else if (v->button_left) {
                newperturb = mjPERT_ROTATE;
            }
            // perturbation onset: reset reference
            if (newperturb > 0 && v->pert.active == 0) {
                mjv_initPerturb_fp(v->m, v->d, &v->scn, &v->pert);
            }
        } else {
            // Enable vis perturb force again
            v->opt.flags[11] = 1;
        }
    }
    v->pert.active = newperturb;

    // detect double-click (250 msec)
    time_t curr_time = time(0);
    if (act == GLFW_PRESS && (curr_time - v->lastclicktm < 0.25) && (button == v->lastbutton)) {
        // determine selection mode
        int selmode = 2;    // Right Click
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            selmode = 1;
        } else if (mods == GLFW_MOD_CONTROL) {
            selmode = 3; // CTRL + Right Click
        }
        // get current window size
        int width, height;
        glfwGetWindowSize_fp(window, &width, &height);
        // find geom and 3D click point, get corresponding body
        mjtNum selpnt[3];

        int selgeom = 0;
        int selskin = 0;
        mjtNum aspectratio = (mjtNum) width / height;
        mjtNum relx = (mjtNum) x / width;
        mjtNum rely = (mjtNum) (height - y) / height;

        int selbody = mjv_select_fp(v->m, v->d, &v->opt,
                                    aspectratio, relx,
                                    rely,
                                    &v->scn, selpnt, &selgeom, &selskin);
        // set lookat point, start tracking is requested
        if (selmode == 2 || selmode == 3) {
            // copy selpnt if geom clicked
            if (selbody >= 0) {
                memcpy(v->cam.lookat, selpnt, sizeof(v->cam.lookat));
            }

            // switch to tracking camera
            if (selmode == 3 && selbody >= 0) {
                v->cam.type = mjCAMERA_TRACKING;
                //v->cam.trackbodyid = selbody;
                v->cam.trackbodyid = 0;
                v->cam.fixedcamid = -1;
            }
        } else { // set body selection
            if (selbody >= 0) {
                // compute localpos
                mjtNum tmp[3];
                mju_sub3_fp(tmp, selpnt, v->d->qpos+3*selbody);
                mju_mulMatTVec_fp(v->pert.localpos, v->d->xmat+9*selbody, tmp, 3, 3);

                // record selection
                v->pert.select = selbody;
                v->pert.skinselect = selskin;
            } else {
                v->pert.select = 0;
                v->pert.skinselect = -1;
            }
        }

        // stop perturbation on select
        v->pert.active = 0;
    }
    // save info
    if (act == GLFW_PRESS) {
        v->lastbutton = button;
        v->lastclicktm = time(0);
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    (void)scancode;
    robot_vis_t* v = glfwGetWindowUserPointer_fp(window);
    if (action == GLFW_RELEASE) {
        return;
    } else if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_P && mods == 0) {
            printf("attaching camera to pelvis\n");
            v->cam.type = mjCAMERA_TRACKING;
            v->cam.trackbodyid = 1;
            v->cam.fixedcamid = -1;
            v->cam.distance = 3;
            v->cam.azimuth = 90;
            v->cam.elevation = -20;
        }
        // control keys
        if (mods == GLFW_MOD_CONTROL) {
            if (key == GLFW_KEY_A) {
                memcpy(v->cam.lookat, v->m->stat.center, sizeof(v->cam.lookat));
                v->cam.distance = 1.5*v->m->stat.extent;
                // set to free camera
                v->cam.type = mjCAMERA_FREE;
            } else if (key == GLFW_KEY_P) {
                printf("qpos: ");
                for (int i = 0; i < v->m->nq; i++) {
                    printf("%f", v->d->qpos[i]);
                    if (i != v->m->nq-1) {
                        printf(", ");
                    }
                }
                printf("\n");
                // mju_printMat_fp(v->d->qpos, v->m->nq, 1);
            } else if (key == GLFW_KEY_Q) {
                glfwSetWindowShouldClose_fp(window, true);
            }
        }
        // toggle visualiztion flag
        for (int i=0; i < mjNVISFLAG; i++) {
            if (key == VISSTRING[i][2][0]) {
                mjtByte flags[mjNVISFLAG];
                memcpy(flags, v->opt.flags, sizeof(flags));
                flags[i] = flags[i] == 0 ? 1 : 0;
                memcpy(v->opt.flags, flags, sizeof(v->opt.flags));
                return;
            }
        }
        // toggle rendering flag
        for (int i=0; i < mjNRNDFLAG; i++) {
            if (key == RNDSTRING[i][2][0]) {
                mjtByte flags[mjNRNDFLAG];
                memcpy(flags, v->scn.flags, sizeof(flags));
                flags[i] = flags[i] == 0 ? 1 : 0;
                memcpy(v->scn.flags, flags, sizeof(v->scn.flags));
                return;
            }
        }
        // toggle geom/site group
        for (int i=0; i < mjNGROUP; i++) {
            if (key == i + 48) {    // Int('0') = 48
                if (mods && GLFW_MOD_SHIFT == true) {
                    mjtByte sitegroup[mjNGROUP];
                    memcpy(sitegroup, v->opt.sitegroup, sizeof(sitegroup));
                    sitegroup[i] = sitegroup[i] > 0 ? 0 : 1;
                    // memcpy(v->opt.sitegroup = sitegroup
                    v->opt.sitegroup[i] = sitegroup[i];
                    return;
                } else {
                    mjtByte geomgroup[mjNGROUP];
                    memcpy(geomgroup, v->opt.geomgroup, sizeof(geomgroup));
                    geomgroup[i] = geomgroup[i] > 0 ? 0 : 1;
                    memcpy(v->opt.geomgroup, geomgroup, sizeof(v->opt.geomgroup));
                    return;
                }
            }
        }
        switch (key) {
            case GLFW_KEY_F1: {     // help
                v->showhelp += 1;
                if (v->showhelp > 1) {
                    v->showhelp = 0;
                }
            } break;
            case GLFW_KEY_F2: {     // option
                v->showoption = !v->showoption;
            } break;
            case GLFW_KEY_F3: {     // info
                v->showinfo = !v->showinfo;
            } break;
            case GLFW_KEY_F4: {     // GRF
                v->showGRF = !v->showGRF;
            } break;
            case GLFW_KEY_F5: {     // toggle fullscreen
                v->showfullscreen = !v->showfullscreen;
                v->showfullscreen ? glfwMaximizeWindow_fp(window) : glfwRestoreWindow_fp(window);
            } break;
            case GLFW_KEY_F7: {     // sensor figure
                v->showsensor = !v->showsensor;
            } break;
            case GLFW_KEY_ENTER: {  // slow motion
                v->slowmotion = !v->slowmotion;
                v->slowmotion ? printf("Slow Motion Mode!\n") : printf("Normal Speed Mode!\n");
            } break;
            case GLFW_KEY_SPACE: {  // pause
                v->paused = !v->paused;
                v->paused ? printf("Paused\n") : printf("Running\n");
            } break;
            case GLFW_KEY_BACKSPACE: {  // reset
                double qpos_init[13] =
                        {-0.04892201261633587, 0.137070621911611, 0.4259146556246555,
                         0.9947601, -0.0483187, -0.0857336, 0.0277026,
                         0.1324580211741131, 0.8517430801704898, -0.9228175102197048,
                         0.07249285764925784, 0.5326225286226256, -0.8761352725538367};
                double qvel_zero[12] = {0};
                mj_resetData_fp(v->m, v->d);
                mju_copy_fp(v->d->qpos, qpos_init, 13);
                mju_copy_fp(v->d->qvel, qvel_zero, v->m->nv);
                v->d->time = 0.0;
                mj_forward_fp(v->m, v->d);
            } break;
            case GLFW_KEY_RIGHT: {      // step forward
                if (v->paused) {
                    mj_step_fp(v->m, v->d);
                }
            } break;
                // case GLFW_KEY_LEFT: {       // step backward
                //     if (v->paused) {
                //         double dt = v->m->opt.timestep;
                //         v->m->opt.timestep = -dt;
                //         mj_step_fp(v->m, v->d);
                //         v->m->opt.timestep = dt;
                //     }
                // } break;
            case GLFW_KEY_DOWN: {      // step forward 100
                if (v->paused) {
                    for (int i = 0; i < 100; i++) {
                        mj_step_fp(v->m, v->d);
                    }
                }
            } break;
                // case GLFW_KEY_UP: {       // step back 100
                //     if (v->paused) {
                //         double dt = v->m->opt.timestep;
                //         v->m->opt.timestep = -dt;
                //         for (int i = 0; i < 100; i++) {
                //             mj_step_fp(v->m, v->d);
                //         }
                //         v->m->opt.timestep = dt;
                //     }
                // } break;
            case GLFW_KEY_ESCAPE: {     // free camera
                v->cam.type = mjCAMERA_FREE;
            } break;
            case GLFW_KEY_EQUAL: {      // bigger font
                if (fontscale < 200) {
                    fontscale += 50;
                    mjr_makeContext_fp(v->m, &v->con, fontscale);
                }
            } break;
            case GLFW_KEY_MINUS: {      // smaller font
                if (fontscale > 100) {
                    fontscale -= 50;
                    mjr_makeContext_fp(v->m, &v->con, fontscale);
                }
            } break;
            case GLFW_KEY_LEFT_BRACKET: {  // '[' previous fixed camera or free
                int fixedcam = v->cam.type;
                if (v->m->ncam > 0 && fixedcam == mjCAMERA_FIXED) {
                    int fixedcamid = v->cam.fixedcamid;
                    if (fixedcamid  > 0) {
                        v->cam.fixedcamid = fixedcamid-1;
                    } else {
                        v->cam.type = mjCAMERA_FREE;
                    }
                }
            } break;
            case GLFW_KEY_RIGHT_BRACKET: {  // ']' next fixed camera
                if (v->m->ncam > 0) {
                    int fixedcam = v->cam.type;
                    int fixedcamid = v->cam.fixedcamid;
                    if (fixedcam != mjCAMERA_FIXED) {
                        v->cam.type = mjCAMERA_FIXED;
                    } else if (fixedcamid < v->m->ncam - 1) {
                        v->cam.fixedcamid = fixedcamid+1;
                    }
                }
            } break;
        }
    }
}

robot_vis_t *robot_vis_init(robot_sim_t* c, const char* modelfile) {
    // Make sure MuJoCo is initialized and the model is loaded
    if (!mujoco_initialized) {
        printf("vis mujoco not init\n");
        if (!robot_mujoco_init(modelfile)) {
            printf("mujoco not init\n");
            return NULL;
        }
    }

    if (!glfw_initialized) {
        printf("glfw not init\n");
        return NULL;
    }
    // Allocate visualization structure
    robot_vis_t *v = malloc(sizeof (robot_vis_t));
    // Set interaction ctrl vars
    v->lastx = 0.0;
    v->lasty = 0.0;
    v->button_left = false;
    v->button_middle = false;
    v->button_right = false;
    v->lastbutton = GLFW_MOUSE_BUTTON_1;
    v->lastclicktm = 0.0;
    // GLFWvidmode* vidmode = glfwGetVideoMode_fp(glfwGetPrimaryMonitor_fp());
    v->refreshrate = glfwGetVideoMode_fp(glfwGetPrimaryMonitor_fp())->refreshRate;
    v->showhelp = 0;
    v->showoption = false;
    v->showGRF = false;
    v->GRFcount = 0;
    v->showfullscreen = false;
    v->showsensor = false;
    v->slowmotion = false;
    v->showinfo = true;
    v->paused = false;
    v->framenum = 0;
    v->lastframenum = 2;
    v->m = c->m;
    v->d = c->d;
    v->marker_num = 0;
    v->perturb_body = 1;
    memset(v->perturb_force, 0.0, 6*sizeof(double));

    // Create window
    v->window = glfwCreateWindow_fp(1200, 900, "robot", NULL, NULL);
    glfwMakeContextCurrent_fp(v->window);
    glfwSwapInterval_fp(0);

    //printf("Refresh Rate: %i\n", v->refreshrate);
    //printf("Resolution: %ix%i\n", 1200, 900);

    // sensorinit(v);
    // grfinit(v);
    // Set up mujoco visualization objects
    // v->cam.type = mjCAMERA_FIXED;
    // v->cam.fixedcamid = 0;
    mjv_defaultCamera_fp(&v->cam);
    mjv_defaultOption_fp(&v->opt);
    v->opt.flags[11] = 1;//v->opt.flags[12];    // Render applied forces
    mjr_defaultContext_fp(&v->con);
    mjv_defaultScene_fp(&v->scn);
    mjv_makeScene_fp(c->m, &v->scn, 1000);
    mjr_makeContext_fp(c->m, &v->con, fontscale);

    // Set callback for user-initiated window close events
    glfwSetWindowUserPointer_fp(v->window, v);
    glfwSetWindowCloseCallback_fp(v->window, window_close_callback);

    // Set glfw callbacks
    glfwSetCursorPosCallback_fp(v->window, mouse_move);
    glfwSetMouseButtonCallback_fp(v->window, mouse_button);
    glfwSetScrollCallback_fp(v->window, scroll);
    glfwSetKeyCallback_fp(v->window, key_callback);

    // show frame axis
    v->opt.frame = 0;
    return v;
}

void robot_vis_close(robot_vis_t *v)
{
    if (!glfw_initialized || !v || !v->window)
        return;

    // Free mujoco objects
    mjv_freeScene_fp(&v->scn);
    mjr_freeContext_fp(&v->con);

    // Close window
    glfwDestroyWindow_fp(v->window);
    v->window = NULL;
}

bool robot_vis_draw(robot_vis_t *v, robot_sim_t *c)
{

    (void)c;
    if (!glfw_initialized)
        return false;

    // Return early if window is closed
    if (!v || !v->window)
        return false;

    // Check if window should be closed
    if (glfwWindowShouldClose_fp(v->window)) {
        robot_vis_close(v);
        return false;
    }
    // If we are rendering to file then force the window to be the right size
    if( v->pipe_video_out != NULL){
        glfwSetWindowSize_fp(v->window, v->video_width, v->video_height);
    }
    // clear old perturbations, apply new
    mju_zero_fp(v->d->xfrc_applied, 6 * v->m->nbody);
    if (v->pert.select > 0) {
        mjv_applyPerturbPose_fp(v->m, v->d, &v->pert, 0); // move mocap bodies only
        mjv_applyPerturbForce_fp(v->m, v->d, &v->pert);
    }
    // Add applied forces to qfrc array
    for (int i = 0; i < 6; i++) {
        v->d->xfrc_applied[6*v->perturb_body + i] += v->perturb_force[i];
    }
    mj_forward_fp(v->m, v->d);
    // Reset xfrc applied to zero
    memset(v->perturb_force, 0, 6*sizeof(double));
    // Set up for rendering
    glfwMakeContextCurrent_fp(v->window);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize_fp(v->window, &viewport.width, &viewport.height);
    //mjrRect smallrect = viewport;
    // Render scene
    mjv_updateScene_fp(c->m, c->d, &v->opt, &v->pert, &v->cam, mjCAT_ALL, &v->scn);

    // Add markers (custom geoms) at end of populated geom list
    // add_vis_markers(v);

    // render
    mjr_render_fp(viewport, &v->scn, &v->con);

    /*if (v->showsensor) {
        if (!v->paused) {
            sensorupdate(v);
        }
        sensorshow(v, smallrect);
    }
    if (v->showGRF) {
        if (!v->paused) {
            grfupdate(v);
        }
        grfshow(v, smallrect);
    }
    if (v->showhelp) {
        mjr_overlay_fp(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, help_title, help_content, &v->con);
    }*/
    if (v->showinfo) {
        char buf[1024];
        char str_slow[20];
        if (v->slowmotion) {
            strcpy(str_slow, "(10x slowdown)");
        } else {
            strcpy(str_slow, "");
        }
        char str_paused[50];
        if(v->paused) {
            strcpy(str_paused, "\nPaused");
        } else {
            strcpy(str_paused, "\nRunning");
        }
        strcat(str_paused, "\nTime:");
        char status[50];
        sprintf(status, "\n\n%.2f", v->d->time);
        strcpy(buf, str_slow);
        strcat(buf, status);
        // status = str_slow * status

        mjr_overlay_fp(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                       str_paused,
                       buf, &v->con);
    }

    // Show updated scene
    glfwSwapBuffers_fp(v->window);
    glfwPollEvents_fp();

    return true;
}

void body_sim_fixed(robot_sim_t *sim){
    // 固定机器人位置姿态

    double body_init[7] = {0.0, 0.0, 1,  1.0, 0.0, 0.0, 0.0};
    mju_copy_fp(&sim->d->qpos[0], body_init, 7);
    //sim->d->xpos = {0,0,1};
    //mju_copy_fp(sim->d->xpos, body_init, 3);
}

int mj_main_init(robot_sim_t *sim)
{
    // 初始化机器人位置姿态
    double body_init[7] = {0.0, 0.0, 0.65, 1.0, 0.0, 0.0, 0.0};
    mju_copy_fp(&sim->d->qpos[0], body_init, 7);
    // 初始化关节角度
    double q_init[6] = {
                                 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0
                                };
    mju_copy_fp(&sim->d->qpos[7], q_init, 6);

    ID_NAME_LOOKUP(sim->m, left_foot_body_id, mjOBJ_BODY, L_Foot);
    ID_NAME_LOOKUP(sim->m, right_foot_body_id, mjOBJ_BODY, R_Foot);
    printf("number of generalized coordinates nq = %d \n", sim->m->nq);
    printf("number of actuators/controls nu = %d \n", sim->m->nu);
    printf("number of degrees of freedom nv = %d \n", sim->m->nv);
    printf("time step = %f \n", sim->m->opt.timestep);

    printf("mujoco simulation init is ok!\n");
}

void body_sim_hold(robot_sim_t *sim){
    // Set stiffness/damping for body translation joints
    //body_sim_fixed(sim);
    for (short i = 0; i < 3; ++i) {
        sim->m->jnt_stiffness[i] = 1e5;
        sim->m->dof_damping[i] = 1e4;
        sim->m->qpos_spring[i] = sim->d->qpos[i];
    }
    // Set damping for body rotation joint
    for (short i = 3; i < 6; ++i)
        sim->m->dof_damping[i] = 1e3;
}

void robot_sim_foot_forces(const robot_sim_t *sim, double cfrc[12])
{
    double force_torque[6];
    double force_global[3];

    // Zero the output foot forces
    mju_zero_fp(cfrc, 12);
    printf("****************\n");
    // Accumulate the forces on each foot
    for (int i = 0; i < sim->d->ncon; ++i) {
        // Get body IDs for both geoms in the collision
        int body1 = sim->m->geom_bodyid[sim->d->contact[i].geom1];
        int body2 = sim->m->geom_bodyid[sim->d->contact[i].geom2];
        // Left foot
        if (body1 == left_foot_body_id || body2 == left_foot_body_id) {
            // Get contact force in world coordinates
            mj_contactForce_fp(sim->m, sim->d, i, force_torque);
            mju_rotVecMatT_fp(force_global, force_torque,
                              sim->d->contact[i].frame);
            // Add to total forces on foot
            if (body1 == left_foot_body_id)
                for (int j = 0; j < 3; ++j) cfrc[j] -= force_global[j];
            else
                for (int j = 0; j < 3; ++j) cfrc[j] += force_global[j];
        }

        // Right foot
        if (body1 == right_foot_body_id || body2 == right_foot_body_id) {
            // Get contact force in world coordinates
            mj_contactForce_fp(sim->m, sim->d, i, force_torque);
            mju_rotVecMatT_fp(force_global, force_torque,
                              sim->d->contact[i].frame);

            // Add to total forces on foot
            if (body1 == right_foot_body_id)
                for (int j = 0; j < 3; ++j) cfrc[j+6] -= force_global[j];
            else
                for (int j = 0; j < 3; ++j) cfrc[j+6] += force_global[j];
        }
    }
}

void step(robot_sim_t *sim)
{
    mj_step_fp(sim->m, sim->d);
}

void robot_sim_set_pd_control(robot_sim_t *sim, double target_pos[16], double p_gain[6], double d_gain[6])
{
    double motor_input[16];
    int dir;

    for (int i = 0; i < 16; ++i)
    {
        // sim->d->qpos[i+7] = 0;
        TARGET_q[i] = target_pos[i];
        motor_input[i] = p_gain[i]*(target_pos[i] - sim->d->qpos[i+7]) +
                         d_gain[i]*(0 - sim->d->qvel[i+7]);
    }
    for (int i = 0; i<16; ++i)
    {
        const char* joint_name = mj_id2name_fp(sim->m, mjOBJ_JOINT, i+4);
        //printf("joint id = %d joint %s  =  %f  input u = %f\n", i, joint_name, sim->d->qpos[i+7], motor_input[i]);
    }
    mju_copy_fp(sim->d->ctrl, motor_input, 16);
}

void robot_step(robot_sim_t *sim, robot_state *bus)
{
    bus->robot_sim_data.sim_time = *robot_sim_time(sim);
    step(sim);
}
void unpack_ctrl_in(const unsigned char *bytes, robot_ctrl *bus)
{
    union byte8 tmp_data;
    int byte_counter = 0;

    for (int j = 0; j < 8; ++j) {
        tmp_data.byte[j] = bytes[byte_counter++];
    }
    bus->robot_ctrl_data.target_q = tmp_data.fdata;

}

void pack_state_out(const robot_state *bus, unsigned char *bytes)
{
    union byte8 tmp_data;
    int byte_counter = 0;

    tmp_data.fdata = bus->robot_sim_data.sim_time;
    for (int j = 0; j < 8; ++j)
    {
        bytes[byte_counter++] = tmp_data.byte[j];
    }

}

double* robot_sim_xquat(robot_sim_t *c, const char* name)
{
    int body_id = mj_name2id_fp(c->m, mjOBJ_BODY, name);
    return &(c->d->xquat[4*body_id]);
}

double* robot_sim_xpos(robot_sim_t *c, const char* name)
{
    int body_id = mj_name2id_fp(initial_model, mjOBJ_BODY, name);
    return &(c->d->xpos[3*body_id]);
}

//***************************
//This function is called at a set frequency, put data here
void save_data(FILE *fid, robot_sim_t *c, double cfrc[12])
{
    //data here should correspond to headers in init_save_data()
    //seperate data by a space %f followed by space
    fprintf(fid,"%f, ", c->d->time);
    fprintf(fid,"%f,",  TARGET_q[4]);
    fprintf(fid,"%f,",  c->d->qpos[11]);
    fprintf(fid,"%f,",  cfrc[0]);
    fprintf(fid,"%f,",  cfrc[1]);
    fprintf(fid,"%f,",  cfrc[2]);


    //Don't remove the newline
    fprintf(fid,"\n");
}

void tt(robot_sim_t *c)
{c->m;}

