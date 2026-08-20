#define PI 3.14159265359f
#define P_CMD_MIN 0.0f
#define P_CMD_MAX 6.283185f  // 359.9999 deg

#define P_FB_MIN -15 * 2 * PI
#define P_FB_MAX 15 * 2 * PI  // rad

#define V_MIN -45.0f
#define V_MAX 45.0f

#define T_MIN -400.0f
#define T_MAX 400.0f

// Backlash feedback: MU150 encoder angle minus motor electrical-angle estimate.
// The value is transported as an unsigned, normalized 12-bit quantity.
#define MU150_CAN_DIFF_MIN_DEG -3.0f
#define MU150_CAN_DIFF_MAX_DEG 3.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f

#define KI_MIN 0.0f
#define KI_MAX 10.0f

#define KD_MIN 0.0f
#define KD_MAX 5.0f
