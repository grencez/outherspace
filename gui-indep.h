
    /* Include the .c file!*/

#include "motion.h"
#include "pnm-image.h"
#include "testcase.h"

#include <math.h>

#define NRacersMax 10

static const bool FollowRacer = true;

static const bool RenderDrawsPattern = false;
static const bool ForceFauxFishEye = false;
static const bool ShowFrameRate = false;
static const bool LightAtCamera = false;
static const bool ShowSpeed = false;

typedef struct motion_input_struct MotionInput;
typedef struct race_craft_struct RaceCraft;
typedef struct pilot_struct Pilot;


struct motion_input_struct
{
    real vert;
    real horz;
    real drift;
    real stride[NDimensions];
    real thrust[2];
    real view_azimuthcc;
    real view_zenith;
    bool boost;
    bool inv_vert;
    bool use_roll;
    bool firing[2];
};

struct race_craft_struct
{
    real health;
    uint pilot_idx;
};

struct pilot_struct
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

        /* Not sure if these will be used evar.*/
    uint mouse_coords[2];
    uint mouse_diff[2];
};

    /* Global state changes.*/
static uint ncheckplanes = 0;
static Plane* checkplanes;
static Point* checkpoints;
static uint nracers = 0;
static uint npilots = 1;
static uint kbd_pilot_idx = 0;
static ObjectMotion racer_motions[NRacersMax];  /* Reset every frame.*/
static RaceCraft crafts[NRacersMax];
static Pilot pilots[NRacersMax];
static uint view_nrows = 400;
static uint view_ncols = 400;
static bool needs_recast = true;
static bool continue_running = true;
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
render_pattern (byte* data, uint nrows, uint ncols, uint stride);
static void
render_RaySpace (RaySpace* space, byte* data,
                 uint nrows, uint ncols, uint stride,
                 real frame_t1);

