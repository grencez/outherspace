
    /* Include the .c file!*/

#ifdef DistribCompute
#include "compute.h"
#endif

#include "dynamic-setup.h"
#include "motion.h"
#include "pnm-image.h"
#include "point.h"
#include "serial.h"
#include "testcase.h"
#include "xfrm.h"

#include <math.h>

#define NRacersMax 10

#ifdef UseRaceCraft
static const bool FollowRacer = true;
#else
static const bool FollowRacer = false;
#endif

static const bool RenderDrawsPattern = false;
static const bool ForceFauxFishEye = false;
static const bool ShowFrameRate = false;
static const bool ShowSpeed = false;

typedef struct MotionInput MotionInput;
typedef struct RaceCraft RaceCraft;
typedef struct Pilot Pilot;


struct MotionInput
{
    real vert;
    real horz;
    real drift;
    real stride[NDimensions];
    real thrust[2];
    real view_azimuthcc;
    real view_zenith;
    bool light_to_camera;
    bool boost;
    bool inv_vert;
    bool use_roll;
    bool firing[2];
    bool lock_drift;
    bool mouse_orbit;
    bool mouse_zoom;
    bool mouse_pan;
    real orbit[2];
    real zoom;
    real pan[2];
};

struct RaceCraft
{
    real health;
    uint pilot_idx;
};

struct Pilot
{
    uint craft_idx;
    MotionInput input;
    Point view_origin;
    PointXfrm view_basis;

    uint image_start_row;
    uint image_start_col;
    RayImage ray_image;

    real stride_magnitude;
    real view_angle;
    real view_width;

        /* Offsets from racer centroid when stationary.*/
    real up_offset;
    real forward_offset;

    Point orbit_focus;
    bool mouse_down;
    int mouse_coords[2];
};

    /* Global state changes.*/
static Track track;
static uint nracers = 0;
static uint npilots = 1;
static uint kbd_pilot_idx = 0;
static ObjectMotion racer_motions[NRacersMax];  /* Reset every frame.*/
static RaceCraft crafts[NRacersMax];
static uint view_nrows = 400;
static uint view_ncols = 400;
static uint npixelzoom = 1;
static real frame_t0 = 0;
static real framerate_report_dt = 0;
static uint framerate_report_count = 0;
    /* Object indices.*/
static uint laser_objidcs[NRacersMax][2];
static uint checkplane_objidx;


static void
init_MotionInput (MotionInput* mot);
static void
init_RaceCraft (RaceCraft* craft);
static void
init_Pilot (Pilot* p);
static void
cleanup_Pilot (Pilot* p);
static void
sync_Pilot (Pilot* gui, Pilot* bkg);
static void
resize_pilot_viewports (uint nrows, uint ncols);
static void
update_object_motion (ObjectMotion* motion, const MotionInput* input);
static void
update_camera_location (Pilot* pilot, const MotionInput* input, real dt);
static void
update_view_params (Pilot* pilot,
                    const MotionInput* input,
                    const RaySpace* space);
static void
set_checkpoint_light (PointLightSource* light,
                      ObjectRaySpace* object,
                      const ObjectMotion* mot,
                      const Point* view_origin);
static void
relative_laser_origin (Point* origin, uint side,
                       const ObjectRaySpace* object);
static void
setup_laser_scenes (RaySpace* space);
static void
update_health (const RaySpace* space, real dt);
static void
render_pattern (byte* data, uint nrows, uint ncols, uint stride,
                uint32 argb_map);
static void
render_RayImage (byte* data, uint nrows, uint ncols, uint stride,
                 const RayImage* ray_image,
                 uint image_start_row,
                 uint image_start_col,
                 uint32 argb_map);
static void
render_pilot_images (byte* data, uint nrows, uint ncols, uint stride,
                     bool argb_order);
static void
update_pilot_images (RaySpace* space, real frame_t1);
static void
init_ui_data (RaySpace* space,
             const Pilot* input_pilots,
             const char* inpathname);
static void
cleanup_ui_data ();

#ifdef SupportOpenGL
    /* This is found in gui-opengl.c.*/
static void
ogl_redraw (const RaySpace* space, uint pilot_idx);
#endif

